import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import socket
import math

class RoverMappingBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge_node')
        
        # ROS2 Publishers
        self.pub_front = self.create_publisher(Range, '/sensor/front', 10)
        self.pub_left  = self.create_publisher(Range, '/sensor/left',  10)
        self.pub_right = self.create_publisher(Range, '/sensor/right', 10)
        self.pub_odom  = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcasters
        self.tf_broadcaster        = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Network Configuration
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 5000))
        self.sock.setblocking(False)
        
        # Latest sensor state
        self.current_yaw       = 0.0
        self.current_left_cm   = 40.0
        self.current_front_cm  = 40.0
        self.current_right_cm  = 40.0
        
        # Dead Reckoning state
        self.x         = 0.0
        self.y         = 0.0
        self.last_time = self.get_clock().now()

        # ── Timeout tracking ─────────────────────────────────────────────────
        # If no UDP packet received for >0.5s, robot is off or stopped
        self.last_packet_time = self.get_clock().now()
        self.PACKET_TIMEOUT   = 0.5   # seconds
        
        self.publish_static_sensor_transforms()
        
        self.timer = self.create_timer(0.05, self.bridge_tick)
        self.get_logger().info("🚀 Telemetry Mapping Bridge initialized on Port 5000.")

    def publish_static_sensor_transforms(self):
        transforms = []

        t_front = TransformStamped()
        t_front.header.frame_id    = 'base_link'
        t_front.child_frame_id     = 'front_sensor_link'
        t_front.transform.translation.x = 0.1
        t_front.transform.translation.y = 0.0
        t_front.transform.translation.z = 0.0
        t_front.transform.rotation.w    = 1.0
        transforms.append(t_front)

        t_left = TransformStamped()
        t_left.header.frame_id    = 'base_link'
        t_left.child_frame_id     = 'left_sensor_link'
        t_left.transform.translation.x = 0.05
        t_left.transform.translation.y = 0.05
        t_left.transform.translation.z = 0.0
        t_left.transform.rotation.z    = math.sin(math.radians(90) / 2)
        t_left.transform.rotation.w    = math.cos(math.radians(90) / 2)
        transforms.append(t_left)

        t_right = TransformStamped()
        t_right.header.frame_id    = 'base_link'
        t_right.child_frame_id     = 'right_sensor_link'
        t_right.transform.translation.x = 0.05
        t_right.transform.translation.y = -0.05
        t_right.transform.translation.z = 0.0
        t_right.transform.rotation.z    = math.sin(math.radians(-90) / 2)
        t_right.transform.rotation.w    = math.cos(math.radians(-90) / 2)
        transforms.append(t_right)

        self.static_tf_broadcaster.sendTransform(transforms)

    def bridge_tick(self):
        """Drain UDP buffer, then publish. Stops odometry if no packets received."""

        packet_received = False

        # Step A: Drain all pending UDP packets
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                payload = data.decode('utf-8')
                parts   = payload.split(',')
                if len(parts) >= 4:
                    self.current_yaw       = float(parts[0])
                    self.current_left_cm   = float(parts[1])
                    self.current_front_cm  = float(parts[2])
                    self.current_right_cm  = float(parts[3])
                    self.last_packet_time  = self.get_clock().now()  # ← update timestamp
                    packet_received        = True
        except BlockingIOError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error parsing packet: {e}")

        current_time = self.get_clock().now()

        # Step B: Check timeout — if no packet for >0.5s, freeze odometry
        time_since_packet = (current_time - self.last_packet_time).nanoseconds / 1e9
        if time_since_packet > self.PACKET_TIMEOUT:
            # Robot is off or disconnected — publish static TF but don't move
            self._publish_static_tf(current_time)
            return

        # Step C: Normal publish cycle
        self.update_and_publish_odom(self.current_yaw, current_time, self.current_front_cm)
        self.pub_front.publish(self.create_range_msg(self.current_front_cm, 'front_sensor_link', current_time))
        self.pub_left.publish(self.create_range_msg(self.current_left_cm,   'left_sensor_link',  current_time))
        self.pub_right.publish(self.create_range_msg(self.current_right_cm, 'right_sensor_link', current_time))

    def _publish_static_tf(self, current_time):
        """Publish TF at current position without moving — keeps RViz happy."""
        qz = math.sin(math.radians(self.current_yaw) * 0.5)
        qw = math.cos(math.radians(self.current_yaw) * 0.5)
        t  = TransformStamped()
        t.header.stamp    = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)

    def update_and_publish_odom(self, yaw_deg, current_time, front_cm):
        yaw_rad = math.radians(yaw_deg)
        dt      = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Mirror ESP32 navigation matrix for velocity estimation
        CRITICAL_DISTANCE = 12.0
        OBSTACLE_DISTANCE = 25.0

        if 0 < front_cm < CRITICAL_DISTANCE:
            v = -0.10
        elif 0 < front_cm < OBSTACLE_DISTANCE:
            v = 0.0
        else:
            v = 0.10   # tuned down from 0.22 for new TT motors

        self.x += v * math.cos(yaw_rad) * dt
        self.y += v * math.sin(yaw_rad) * dt

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp          = current_time.to_msg()
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        self.pub_odom.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp          = current_time.to_msg()
        t.header.frame_id       = 'odom'
        t.child_frame_id        = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z    = sy
        t.transform.rotation.w    = cy
        self.tf_broadcaster.sendTransform(t)

    def create_range_msg(self, cm_val, frame_id, current_time):
        msg = Range()
        msg.header.stamp     = current_time.to_msg()
        msg.header.frame_id  = frame_id
        msg.radiation_type   = 0
        msg.field_of_view    = 0.5
        msg.min_range        = 0.02
        msg.max_range        = 4.0
        msg.range            = cm_val / 100.0
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = RoverMappingBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()