import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0, y0, x1, y1):
    """Return grid cells along the ray from (x0, y0) to (x1, y1), excluding endpoint."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while (x, y) != (x1, y1):
        cells.append((x, y))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        # Grid parameters: 10m x 10m local map at 5cm resolution
        self.resolution = 0.05
        self.width = 200
        self.height = 200
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0
        # Log-odds probability parameters
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.l_min = -4.0
        self.l_max = 5.0
        self.l_lidar_occ = 0.85
        self.l_lidar_free = -0.35
        self.l_us_occ = 0.65
        self.l_us_free = -0.20
        # TF2 buffer for sensor-to-base_link transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Subscribers
        self.create_subscription(LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.create_subscription(Range, '/ultrasonic/center', self.range_callback, 10)
        self.create_subscription(Range, '/ultrasonic/left', self.range_callback, 10)
        self.create_subscription(Range, '/ultrasonic/right', self.range_callback, 10)
        # Match RViz2 Map QoS (Reliable + Transient Local)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/fused_local_map', map_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/fused_scan', 10)
        # Publish fused outputs at 10 Hz
        self.create_timer(0.1, self.publish_fused_outputs)
        self.get_logger().info('Bayesian Sensor Fusion Node initialized.')

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return gx, gy
        return None

    def get_sensor_pose_2d(self, frame_id):
        try:
            tf = self.tf_buffer.lookup_transform('base_link', frame_id, rclpy.time.Time())
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            tyaw = yaw_from_quaternion(tf.transform.rotation)
            return tx, ty, tyaw
        except TransformException:
            return None

    def lidar_callback(self, msg: LaserScan):
        pose = self.get_sensor_pose_2d(msg.header.frame_id)
        if pose is None:
            return
        sx, sy, syaw = pose
        start_cell = self.world_to_grid(sx, sy)
        if start_cell is None:
            return
        self.log_odds *= 0.96
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                world_angle = syaw + angle
                ex = sx + r * math.cos(world_angle)
                ey = sy + r * math.sin(world_angle)
                end_cell = self.world_to_grid(ex, ey)
                if end_cell is not None:
                    for cx, cy in bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1]):
                        self.log_odds[cy, cx] = max(self.l_min, self.log_odds[cy, cx] + self.l_lidar_free)
                    self.log_odds[end_cell[1], end_cell[0]] = min(
                        self.l_max, self.log_odds[end_cell[1], end_cell[0]] + self.l_lidar_occ
                    )
            angle += msg.angle_increment

    def range_callback(self, msg: Range):
        if not math.isfinite(msg.range) or msg.range < msg.min_range or msg.range > msg.max_range:
            return
        pose = self.get_sensor_pose_2d(msg.header.frame_id)
        if pose is None:
            return
        sx, sy, syaw = pose
        start_cell = self.world_to_grid(sx, sy)
        if start_cell is None:
            return
        fov = msg.field_of_view if msg.field_of_view > 0.0 else 0.44
        cone_angles = np.linspace(-fov / 2.0, fov / 2.0, 9)
        for rel_angle in cone_angles:
            weight = math.cos((rel_angle / (fov / 2.0)) * (math.pi / 3.0))
            world_angle = syaw + rel_angle
            ex = sx + msg.range * math.cos(world_angle)
            ey = sy + msg.range * math.sin(world_angle)
            end_cell = self.world_to_grid(ex, ey)
            if end_cell is not None:
                for cx, cy in bresenham(start_cell[0], start_cell[1], end_cell[0], end_cell[1]):
                    self.log_odds[cy, cx] = max(self.l_min, self.log_odds[cy, cx] + self.l_us_free * weight)
                self.log_odds[end_cell[1], end_cell[0]] = min(
                    self.l_max, self.log_odds[end_cell[1], end_cell[0]] + self.l_us_occ * weight
                )

    def publish_fused_outputs(self):
        now = self.get_clock().now().to_msg()
        probs = 1.0 - (1.0 / (1.0 + np.exp(self.log_odds)))
        grid_data = np.full((self.height, self.width), -1, dtype=np.int8)
        known_mask = np.abs(self.log_odds) > 0.15
        grid_data[known_mask] = (probs[known_mask] * 100.0).astype(np.int8)
        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = now
        occ_msg.header.frame_id = 'base_link'
        occ_msg.info.resolution = self.resolution
        occ_msg.info.width = self.width
        occ_msg.info.height = self.height
        occ_msg.info.origin.position.x = self.origin_x
        occ_msg.info.origin.position.y = self.origin_y
        occ_msg.info.origin.orientation.w = 1.0
        occ_msg.data = grid_data.flatten().tolist()
        self.map_pub.publish(occ_msg)
        occ_y, occ_x = np.where(probs > 0.65)
        num_bins = 360
        fused_ranges = np.full(num_bins, float('inf'), dtype=np.float32)
        if len(occ_x) > 0:
            wx = self.origin_x + (occ_x + 0.5) * self.resolution
            wy = self.origin_y + (occ_y + 0.5) * self.resolution
            dists = np.hypot(wx, wy)
            angles = np.arctan2(wy, wx)
            bins = ((angles + math.pi) / (2.0 * math.pi) * num_bins).astype(int) % num_bins
            np.minimum.at(fused_ranges, bins, dists)
        scan_msg = LaserScan()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / num_bins
        scan_msg.range_min = 0.05
        scan_msg.range_max = 5.0
        scan_msg.ranges = [float(r) if r <= 5.0 else float('inf') for r in fused_ranges]
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
