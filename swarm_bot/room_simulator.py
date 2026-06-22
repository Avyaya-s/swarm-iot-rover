"""
room_simulator.py — Hardware-Free Simulation Node
==================================================
Simulates the ESP32 rover moving around a virtual room with obstacles.
Publishes to the EXACT same topics as rover_bridge_node so mapper_node
works identically whether physical hardware is connected or not.

Publishes:
    /odom               nav_msgs/Odometry
    /sensor/front       sensor_msgs/Range
    /sensor/left        sensor_msgs/Range
    /sensor/right       sensor_msgs/Range

Usage (instead of launching rover_bridge_node):
    ros2 run swarm_bot room_simulator

Then launch mapper_node separately:
    ros2 run swarm_bot mapper_node

Room layout (top-down, metres):
    ┌─────────────────────┐  y=3.0
    │                     │
    │      ┌───┐          │
    │      │BOX│          │
    │      └───┘          │
    │                     │
    └─────────────────────┘  y=0.0
  x=0.0                  x=3.0
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

# ── Virtual Room Definition ──────────────────────────────────────────────────
# Each wall is a line segment (x1, y1, x2, y2) in metres
ROOM_WALLS = [
    # Outer walls — 3m × 3m room
    (0.0, 0.0, 3.0, 0.0),   # south wall
    (3.0, 0.0, 3.0, 3.0),   # east wall
    (3.0, 3.0, 0.0, 3.0),   # north wall
    (0.0, 3.0, 0.0, 0.0),   # west wall
    # Inner obstacle — 0.6m × 0.6m box centred at (1.5, 1.5)
    (1.2, 1.2, 1.8, 1.2),   # box south
    (1.8, 1.2, 1.8, 1.8),   # box east
    (1.8, 1.8, 1.2, 1.8),   # box north
    (1.2, 1.8, 1.2, 1.2),   # box west
]

# ── Simulation Parameters ────────────────────────────────────────────────────
STEP_RATE_HZ    = 20.0          # matches ESP32 loop rate
CRUISE_SPEED    = 0.22          # m/s  (matches rover_bridge_node v constant)
REVERSE_SPEED   = -0.15         # m/s  (matches rover_bridge_node)
OBSTACLE_DIST_M = 0.25          # 25 cm — turn threshold
CRITICAL_DIST_M = 0.12          # 12 cm — reverse threshold
MAX_RAY_M       = 4.0           # HC-SR04 max range


class RoomSimulator(Node):

    def __init__(self):
        super().__init__('room_simulator')

        # ── Publishers — identical topic names to rover_bridge_node ──────────
        self.pub_odom  = self.create_publisher(Odometry, '/odom',         10)
        self.pub_front = self.create_publisher(Range,    '/sensor/front',  10)
        self.pub_left  = self.create_publisher(Range,    '/sensor/left',   10)
        self.pub_right = self.create_publisher(Range,    '/sensor/right',  10)

        # ── TF broadcaster (odom → base_link) ────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Robot state ───────────────────────────────────────────────────────
        self.x   = 0.5          # start 50cm in from corner
        self.y   = 0.5
        self.yaw = 0.0          # radians, facing east

        self.last_time = self.get_clock().now()

        # ── Main simulation tick ──────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / STEP_RATE_HZ, self.tick)

        self.get_logger().info(
            '🏠 Room Simulator started. Virtual 3m×3m room with centre obstacle.\n'
            '   Publishing to /odom, /sensor/front, /sensor/left, /sensor/right\n'
            '   Run mapper_node separately to build the occupancy grid.'
        )

    # ── Simulation tick ───────────────────────────────────────────────────────
    def tick(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 1. Cast rays to get sensor distances
        front_m = self._cast_ray(self.x, self.y, self.yaw)
        left_m  = self._cast_ray(self.x, self.y, self.yaw + math.pi / 2)
        right_m = self._cast_ray(self.x, self.y, self.yaw - math.pi / 2)

        # 2. Navigation decision — mirrors ESP32 logic exactly
        if 0 < front_m < CRITICAL_DIST_M:
            # Reverse
            self.x += REVERSE_SPEED * math.cos(self.yaw) * dt
            self.y += REVERSE_SPEED * math.sin(self.yaw) * dt

        elif 0 < front_m < OBSTACLE_DIST_M:
            # Turn toward more open side
            if left_m > right_m:
                self.yaw += math.radians(90)
            else:
                self.yaw -= math.radians(90)
            # Normalise yaw to [-π, π]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        else:
            # Cruise forward
            self.x += CRUISE_SPEED * math.cos(self.yaw) * dt
            self.y += CRUISE_SPEED * math.sin(self.yaw) * dt

        # 3. Clamp robot inside room (safety — shouldn't be needed)
        self.x = max(0.05, min(2.95, self.x))
        self.y = max(0.05, min(2.95, self.y))

        # 4. Publish everything
        self._publish_odom(now)
        self._publish_range(self.pub_front, front_m, 'front_sensor_link', now)
        self._publish_range(self.pub_left,  left_m,  'left_sensor_link',  now)
        self._publish_range(self.pub_right, right_m, 'right_sensor_link', now)

        self.get_logger().info(
            f'x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.0f}° | '
            f'F={front_m*100:.0f}cm L={left_m*100:.0f}cm R={right_m*100:.0f}cm'
        )

    # ── Ray casting — wall intersection ───────────────────────────────────────
    def _cast_ray(self, rx, ry, angle_rad, max_dist=MAX_RAY_M):
        """
        Cast a ray from (rx, ry) at angle_rad.
        Returns distance in metres to nearest wall segment, capped at max_dist.
        Uses parametric ray–segment intersection.
        """
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        min_dist = max_dist

        for (x1, y1, x2, y2) in ROOM_WALLS:
            wall_dx = x2 - x1
            wall_dy = y2 - y1
            denom = dx * wall_dy - dy * wall_dx
            if abs(denom) < 1e-10:
                continue   # ray parallel to wall

            t = ((x1 - rx) * wall_dy - (y1 - ry) * wall_dx) / denom
            u = ((x1 - rx) * dy      - (y1 - ry) * dx)      / denom

            if t > 0.01 and 0.0 <= u <= 1.0:
                min_dist = min(min_dist, t)

        return round(min_dist, 3)

    # ── ROS message publishers ────────────────────────────────────────────────
    def _publish_odom(self, now):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Odometry message
        odom = Odometry()
        odom.header.stamp          = now.to_msg()
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

        # TF transform
        t = TransformStamped()
        t.header.stamp          = now.to_msg()
        t.header.frame_id       = 'odom'
        t.child_frame_id        = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

    def _publish_range(self, publisher, dist_m, frame_id, now):
        msg = Range()
        msg.header.stamp     = now.to_msg()
        msg.header.frame_id  = frame_id
        msg.radiation_type   = Range.ULTRASOUND
        msg.field_of_view    = 0.5
        msg.min_range        = 0.02
        msg.max_range        = MAX_RAY_M
        msg.range            = dist_m
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoomSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()