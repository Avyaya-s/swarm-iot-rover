"""
mapper_node.py — Occupancy Grid Mapper for IoT Level-4 Rover
=============================================================
Subscribes to the ACTUAL topics published by rover_bridge_node:
    /odom               nav_msgs/Odometry     — robot position (x, y, theta)
    /sensor/front       sensor_msgs/Range     — front HC-SR04 (metres)
    /sensor/left        sensor_msgs/Range     — left HC-SR04 (metres)
    /sensor/right       sensor_msgs/Range     — right HC-SR04 (metres)

Publishes:
    /map                nav_msgs/OccupancyGrid  — 2D occupancy grid for RViz

Algorithm: Bresenham line traversal from robot cell to obstacle cell.
    FREE  cells (along ray)  = 0
    OCCUPIED cells (endpoint) = 100
    UNKNOWN (never seen)      = -1

Grid: 200×200 cells at 0.05m/cell = 10m × 10m map, centred on origin.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range
import math

# ── Grid Configuration ──────────────────────────────────────────────────────
GRID_SIZE   = 200          # cells per axis  (200 × 200)
CELL_SIZE   = 0.05         # metres per cell (5 cm)
HALF        = GRID_SIZE // 2   # 100 — origin offset in cells

# ── Sensor mounting angles relative to robot heading ────────────────────────
# (these match the static TF offsets in rover_bridge_node)
SENSOR_ANGLES = {
    'front':  0.0,
    'left':  90.0,
    'right': -90.0,
}

# ── Max range to trust (metres) — beyond this = sensor noise / open space ──
MAX_TRUST_RANGE = 2.50   # 250 cm; HC-SR04 rated to 4m but noisy beyond ~2.5m
MIN_TRUST_RANGE = 0.05   # 5 cm


class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')

        # ── Grid state: flat list, row-major (index = y*W + x) ──────────────
        self.grid = [-1] * (GRID_SIZE * GRID_SIZE)

        # ── Robot pose (updated by /odom callback) ───────────────────────────
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_yaw   = 0.0   # radians

        # ── Latest sensor readings (metres) ─────────────────────────────────
        self.range_front = -1.0
        self.range_left  = -1.0
        self.range_right = -1.0

        # ── QoS: Best Effort to match rover_bridge_node publishers ──────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.sub_odom  = self.create_subscription(Odometry, '/odom',          self.odom_cb,  qos)
        self.sub_front = self.create_subscription(Range,    '/sensor/front',   self.front_cb, qos)
        self.sub_left  = self.create_subscription(Range,    '/sensor/left',    self.left_cb,  qos)
        self.sub_right = self.create_subscription(Range,    '/sensor/right',   self.right_cb, qos)

        # ── Map publisher — TRANSIENT_LOCAL so late subscribers get last map ──
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        # ── Publish map at 2 Hz (no need faster — RViz handles it fine) ─────
        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info(
            f'🗺️  Mapper node ready. Grid: {GRID_SIZE}×{GRID_SIZE} @ {CELL_SIZE*100:.0f}cm/cell '
            f'= {GRID_SIZE*CELL_SIZE:.1f}m × {GRID_SIZE*CELL_SIZE:.1f}m'
        )

    # ── Odom callback: update pose, then immediately ray-trace ───────────────
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion (only z and w matter for planar robot)
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = 2.0 * math.atan2(qz, qw)   # radians

        # Ray-trace with latest sensor readings every time pose updates
        self._update_map()

    # ── Sensor callbacks: just cache the value ────────────────────────────────
    def front_cb(self, msg: Range):
        self.range_front = msg.range

    def left_cb(self, msg: Range):
        self.range_left = msg.range

    def right_cb(self, msg: Range):
        self.range_right = msg.range

    # ── Core mapping logic ────────────────────────────────────────────────────
    def _update_map(self):
        """Called every time odometry updates. Ray-traces all three sensors."""

        # Robot position → grid cell
        rx, ry = self._world_to_cell(self.robot_x, self.robot_y)

        # Mark robot's own cell as free
        self._mark(rx, ry, 0)

        # Ray-trace each sensor
        sensors = [
            (self.range_front, SENSOR_ANGLES['front']),
            (self.range_left,  SENSOR_ANGLES['left']),
            (self.range_right, SENSOR_ANGLES['right']),
        ]

        for dist_m, sensor_angle_deg in sensors:
            if dist_m <= 0:
                continue   # no reading yet

            # Absolute world angle = robot heading + sensor offset
            abs_angle = self.robot_yaw + math.radians(sensor_angle_deg)

            if MIN_TRUST_RANGE < dist_m < MAX_TRUST_RANGE:
                # Normal reading: FREE along ray, OCCUPIED at endpoint
                self._bresenham_ray(rx, ry, dist_m, abs_angle, mark_end_occupied=True)
            else:
                # Out-of-range / max reading: only mark FREE along ray (no wall found)
                self._bresenham_ray(rx, ry, MAX_TRUST_RANGE, abs_angle, mark_end_occupied=False)

    def _bresenham_ray(self, rx, ry, dist_m, angle_rad, mark_end_occupied: bool):
        """
        Traverse the grid from (rx, ry) toward the obstacle at distance dist_m
        along angle_rad using Bresenham's line algorithm.

        Cells along the path → FREE (0)
        Final cell           → OCCUPIED (100)  if mark_end_occupied=True
        """
        # Obstacle endpoint in world coords
        end_x = self.robot_x + dist_m * math.cos(angle_rad)
        end_y = self.robot_y + dist_m * math.sin(angle_rad)

        # Convert to grid cells
        ex, ey = self._world_to_cell(end_x, end_y)

        # Bresenham traversal from (rx,ry) to (ex,ey)
        cells = self._bresenham_cells(rx, ry, ex, ey)

        for i, (cx, cy) in enumerate(cells):
            is_last = (i == len(cells) - 1)
            if is_last and mark_end_occupied:
                # Only upgrade to OCCUPIED — never downgrade a wall to free
                self._mark(cx, cy, 100)
            else:
                # Only mark FREE if cell is currently unknown — don't erase walls
                if self._get(cx, cy) == -1:
                    self._mark(cx, cy, 0)

    @staticmethod
    def _bresenham_cells(x0, y0, x1, y1):
        """Returns list of (x,y) grid cells from (x0,y0) to (x1,y1) inclusive."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x   += sx
            if e2 < dx:
                err += dx
                y   += sy

        return cells

    # ── Grid helpers ─────────────────────────────────────────────────────────
    def _world_to_cell(self, wx, wy):
        """Convert world coordinates (metres) to grid cell indices."""
        cx = int(wx / CELL_SIZE) + HALF
        cy = int(wy / CELL_SIZE) + HALF
        return cx, cy

    def _in_bounds(self, cx, cy):
        return 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE

    def _mark(self, cx, cy, value):
        if self._in_bounds(cx, cy):
            self.grid[cy * GRID_SIZE + cx] = value

    def _get(self, cx, cy):
        if self._in_bounds(cx, cy):
            return self.grid[cy * GRID_SIZE + cx]
        return -1

    # ── Map publisher ─────────────────────────────────────────────────────────
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'          # same frame as robot pose

        msg.info.resolution = CELL_SIZE
        msg.info.width      = GRID_SIZE
        msg.info.height     = GRID_SIZE

        # Origin = bottom-left corner of the grid in world coords
        msg.info.origin.position.x = -(HALF * CELL_SIZE)
        msg.info.origin.position.y = -(HALF * CELL_SIZE)
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()