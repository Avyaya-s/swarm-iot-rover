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
    OCCUPIED cells (endpoint) = 100 (only after 3 consistent hits)
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
GRID_SIZE   = 200
CELL_SIZE   = 0.05
HALF        = GRID_SIZE // 2

# ── Sensor mounting angles relative to robot heading ────────────────────────
SENSOR_ANGLES = {
    'front':  0.0,
    'left':  90.0,
    'right': -90.0,
}

# ── Range filter ─────────────────────────────────────────────────────────────
MAX_TRUST_RANGE    = 1.50   # ignore readings beyond 1.5m (noise)
MIN_TRUST_RANGE    = 0.10   # ignore readings below 10cm
OCCUPIED_THRESHOLD = 3      # cell must be hit 3 times to be marked as wall


class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')

        # ── Grid state ───────────────────────────────────────────────────────
        self.grid      = [-1] * (GRID_SIZE * GRID_SIZE)
        self.hit_count = [0]  * (GRID_SIZE * GRID_SIZE)  # probabilistic counter

        # ── Robot pose ───────────────────────────────────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0

        # ── Sensor readings ──────────────────────────────────────────────────
        self.range_front = -1.0
        self.range_left  = -1.0
        self.range_right = -1.0

        # ── QoS ─────────────────────────────────────────────────────────────
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ── Subscriptions ────────────────────────────────────────────────────
        self.sub_odom  = self.create_subscription(Odometry, '/odom',         self.odom_cb,  qos)
        self.sub_front = self.create_subscription(Range,    '/sensor/front',  self.front_cb, qos)
        self.sub_left  = self.create_subscription(Range,    '/sensor/left',   self.left_cb,  qos)
        self.sub_right = self.create_subscription(Range,    '/sensor/right',  self.right_cb, qos)

        # ── Map publisher — TRANSIENT_LOCAL so late subscribers get last map ─
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.timer = self.create_timer(0.5, self.publish_map)

        self.get_logger().info(
            f'Mapper node ready. Grid: {GRID_SIZE}x{GRID_SIZE} @ {CELL_SIZE*100:.0f}cm/cell '
            f'= {GRID_SIZE*CELL_SIZE:.1f}m x {GRID_SIZE*CELL_SIZE:.1f}m | '
            f'Max range: {MAX_TRUST_RANGE}m | Wall threshold: {OCCUPIED_THRESHOLD} hits'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = 2.0 * math.atan2(qz, qw)
        self._update_map()

    def front_cb(self, msg: Range):
        self.range_front = msg.range

    def left_cb(self, msg: Range):
        self.range_left = msg.range

    def right_cb(self, msg: Range):
        self.range_right = msg.range

    # ── Core mapping ──────────────────────────────────────────────────────────
    def _update_map(self):
        rx, ry = self._world_to_cell(self.robot_x, self.robot_y)
        self._mark_free(rx, ry)

        sensors = [
            (self.range_front, SENSOR_ANGLES['front']),
            (self.range_left,  SENSOR_ANGLES['left']),
            (self.range_right, SENSOR_ANGLES['right']),
        ]

        for dist_m, sensor_angle_deg in sensors:
            if dist_m <= 0:
                continue

            abs_angle = self.robot_yaw + math.radians(sensor_angle_deg)

            if MIN_TRUST_RANGE < dist_m < MAX_TRUST_RANGE:
                # Valid reading — mark free along ray, occupied at end
                self._bresenham_ray(rx, ry, dist_m, abs_angle, mark_end_occupied=True)
            # Readings outside trust range are ignored completely
            # (no long free-space rays causing extended lines)

    def _bresenham_ray(self, rx, ry, dist_m, angle_rad, mark_end_occupied: bool):
        end_x = self.robot_x + dist_m * math.cos(angle_rad)
        end_y = self.robot_y + dist_m * math.sin(angle_rad)
        ex, ey = self._world_to_cell(end_x, end_y)

        cells = self._bresenham_cells(rx, ry, ex, ey)

        for i, (cx, cy) in enumerate(cells):
            is_last = (i == len(cells) - 1)
            if is_last and mark_end_occupied:
                self._mark_occupied(cx, cy)
            else:
                if self._get(cx, cy) == -1:
                    self._mark_free(cx, cy)

    @staticmethod
    def _bresenham_cells(x0, y0, x1, y1):
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

    # ── Grid helpers ──────────────────────────────────────────────────────────
    def _world_to_cell(self, wx, wy):
        cx = int(wx / CELL_SIZE) + HALF
        cy = int(wy / CELL_SIZE) + HALF
        return cx, cy

    def _in_bounds(self, cx, cy):
        return 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE

    def _mark_free(self, cx, cy):
        """Mark cell as free — never overwrites a confirmed wall."""
        if self._in_bounds(cx, cy):
            idx = cy * GRID_SIZE + cx
            if self.grid[idx] != 100:   # don't erase confirmed walls
                self.grid[idx] = 0

    def _mark_occupied(self, cx, cy):
        """Probabilistic wall marking — requires OCCUPIED_THRESHOLD hits."""
        if self._in_bounds(cx, cy):
            idx = cy * GRID_SIZE + cx
            self.hit_count[idx] += 1
            if self.hit_count[idx] >= OCCUPIED_THRESHOLD:
                self.grid[idx] = 100

    def _get(self, cx, cy):
        if self._in_bounds(cx, cy):
            return self.grid[cy * GRID_SIZE + cx]
        return -1

    # ── Map publisher ─────────────────────────────────────────────────────────
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'odom'
        msg.info.resolution           = CELL_SIZE
        msg.info.width                = GRID_SIZE
        msg.info.height               = GRID_SIZE
        msg.info.origin.position.x    = -(HALF * CELL_SIZE)
        msg.info.origin.position.y    = -(HALF * CELL_SIZE)
        msg.info.origin.orientation.w = 1.0
        msg.data                      = self.grid
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()