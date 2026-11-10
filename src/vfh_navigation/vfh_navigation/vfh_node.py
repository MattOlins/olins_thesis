"""ROS 2 adapter for the independently testable navigation components."""

from dataclasses import fields
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import quaternion_yaw, transform_point, wrap_angle
from .grid_map import GridMap
from .models import NavigationConfig, Pose2D, ScanData
from .navigation import Navigator


class VFHNode(Node):
    """Own ROS I/O and provide time-consistent inputs to the navigation core."""

    def __init__(self):
        super().__init__('vfh_node')
        defaults = NavigationConfig()
        for field in fields(defaults):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        for name, value in (
                ('navigation_mode', 'vfh'), ('goal_x', 10.0), ('goal_y', 0.0),
                ('goal_frame', 'odom'), ('odom_frame', 'odom'),
                ('map_frame', 'map'), ('base_frame', 'base_footprint'),
                ('require_map', False)):
            self.declare_parameter(name, value)
        self.config = NavigationConfig(**{
            field.name: self.get_parameter(field.name).value for field in fields(defaults)})
        self.mode = self.get_parameter('navigation_mode').value
        self.require_map = self.get_parameter('require_map').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.navigator = Navigator(self.config, self.mode)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = self.odom = None
        self.scan_received = self.odom_received = None
        self.last_tick = None
        self.map_error = None
        self.map_received = False
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.get_parameter('goal_frame').value
        self.goal.pose.position.x = float(self.get_parameter('goal_x').value)
        self.goal.pose.position.y = float(self.get_parameter('goal_y').value)
        self.goal.pose.orientation.w = 1.0
        self.resolved_goal = None
        self.published_path_revision = -1
        self.last_state = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, '/global_path', retained)
        self.target_pub = self.create_publisher(PoseStamped, '/local_goal', 10)
        self.state_pub = self.create_publisher(DiagnosticArray, '/navigation_state', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, retained)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Navigation started: {self.mode}; awaiting sensor data and TF')

    def scan_callback(self, message):
        """Cache the latest scan and its ROS receipt time."""
        self.scan = message
        self.scan_received = self.get_clock().now().nanoseconds/1e9

    def odom_callback(self, message):
        """Cache odometry without running control inside the subscription."""
        self.odom = message
        self.odom_received = self.get_clock().now().nanoseconds/1e9

    def map_callback(self, message):
        """Copy map data without passing mutable ROS arrays to the planning worker."""
        self.map_received = bool(
            message.header.frame_id == self.map_frame and message.info.width > 0 and
            message.info.height > 0 and
            len(message.data) == message.info.width*message.info.height)
        if self.mode != 'vfh_astar':
            return
        try:
            if message.header.frame_id != self.map_frame:
                raise ValueError('unexpected map frame')
            origin = message.info.origin
            q = origin.orientation
            yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
            if abs(q.x) > 1e-5 or abs(q.y) > 1e-5:
                raise ValueError('map origin must be planar')
            grid = GridMap.from_data(
                message.data, message.info.width, message.info.height,
                message.info.resolution, Pose2D(origin.position.x, origin.position.y, yaw),
                self.config.safe_radius, self.config.free_threshold)
            self.navigator.set_map(grid)
            self.map_error = None
        except ValueError as error:
            self.map_error = f'invalid_map: {error}'
            self.navigator.invalidate('invalid_map')

    def goal_callback(self, message):
        """Replace the final goal and invalidate all previous episode state."""
        coordinates = message.pose.position.x, message.pose.position.y
        if not message.header.frame_id or not all(math.isfinite(v) for v in coordinates):
            self.get_logger().error('Rejected goal with missing frame or nonfinite coordinates')
            return
        self.goal = message
        self.resolved_goal = None
        self.navigator.reset(preserve_obstacles=True)
        self.publish_stop()
        self.publish_path()

    def lookup(self, target, source, stamp=None):
        """Return planar TF without a blocking wait in the control timer."""
        if target == source:
            return Pose2D()
        transform = self.tf_buffer.lookup_transform(target, source, stamp or Time()).transform
        q = transform.rotation
        return Pose2D(transform.translation.x, transform.translation.y,
                      quaternion_yaw(q.x, q.y, q.z, q.w))

    def scan_transform(self):
        """Compensate robot motion between scan acquisition and the odometry sample."""
        transform = self.tf_buffer.lookup_transform_full(
            self.base_frame, Time.from_msg(self.odom.header.stamp),
            self.scan.header.frame_id, Time.from_msg(self.scan.header.stamp),
            self.odom_frame).transform
        q = transform.rotation
        return Pose2D(transform.translation.x, transform.translation.y,
                      quaternion_yaw(q.x, q.y, q.z, q.w))

    def resolve_goal(self):
        """Freeze relative-frame goals once; preserve map/odom goal semantics."""
        if self.resolved_goal is not None:
            return self.resolved_goal
        source = self.goal.header.frame_id
        canonical = self.map_frame if self.mode == 'vfh_astar' else (
            self.map_frame if source == self.map_frame else self.odom_frame)
        transform = self.lookup(canonical, source, Time.from_msg(self.goal.header.stamp))
        point = transform_point((self.goal.pose.position.x, self.goal.pose.position.y), transform)
        self.resolved_goal = canonical, point
        return self.resolved_goal

    def data_is_fresh(self, now):
        """Check receipt and acquisition ages in simulation time."""
        inputs = (self.scan, self.scan_received), (self.odom, self.odom_received)
        for message, received in inputs:
            if message is None or received is None:
                return False
            acquired = Time.from_msg(message.header.stamp).nanoseconds/1e9
            if not (0 <= now-received < self.config.sensor_timeout and
                    -0.05 <= now-acquired < self.config.sensor_timeout):
                return False
        return True

    def publish_stop(self):
        """Publish an immediate zero command."""
        self.navigator.controller.velocity.reset()
        self.cmd_pub.publish(Twist())

    def publish_path(self):
        """Publish accepted routes and explicit invalidations once per revision."""
        if self.published_path_revision == self.navigator.path_revision:
            return
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        route = self.navigator.follower.path
        for index, point in enumerate(route):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = point
            next_point = route[min(index+1, len(route)-1)]
            angle = math.atan2(next_point[1]-point[1], next_point[0]-point[0])
            pose.pose.orientation.z = math.sin(angle/2)
            pose.pose.orientation.w = math.cos(angle/2)
            message.poses.append(pose)
        self.path_pub.publish(message)
        self.published_path_revision = self.navigator.path_revision

    def publish_result(self, result):
        """Publish command, local target, and structured diagnostics at 10 Hz."""
        command = Twist()
        command.linear.x = result.local.command.linear
        command.angular.z = result.local.command.angular
        self.cmd_pub.publish(command)
        if result.target is not None:
            target = PoseStamped()
            target.header.frame_id = self.odom_frame
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y = result.target
            target.pose.orientation.w = 1.0
            self.target_pub.publish(target)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'navigation', 'four_wheel_robot'
        status.level = (DiagnosticStatus.ERROR if result.state == 'FAILED' else
                        DiagnosticStatus.WARN if result.state in ('BLOCKED', 'WAITING') else
                        DiagnosticStatus.OK)
        status.message = result.reason
        histogram = result.local.histogram
        values = {
            'state': result.state, 'reason': result.reason, 'mode': self.mode,
            'remaining_distance': result.remaining, 'progress': result.progress,
            'selected_steering': result.local.selected,
            'observed_clearance': histogram.clearance if histogram else math.nan,
            'replan_count': self.navigator.replan_count,
            'stuck_replans': self.navigator.stuck_replans,
            'turn_reversals': result.turn_reversals,
            'map_revision': self.navigator.map_revision,
            'goal_revision': self.navigator.goal_revision,
            'remembered_obstacles': len(self.navigator.controller.obstacle_memory),
        }
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        diagnostics.status = [status]
        self.state_pub.publish(diagnostics)
        state = result.state, result.reason
        if state != self.last_state:
            self.get_logger().info(f'{result.state}: {result.reason}')
            self.last_state = state
        self.publish_path()

    def control_loop(self):
        """Capture sensor geometry/TF and advance the navigation core once."""
        now = self.get_clock().now().nanoseconds/1e9
        dt = 0.1 if self.last_tick is None else now-self.last_tick
        if dt < 0:
            self.navigator.reset()
            self.tf_buffer.clear()
            self.resolved_goal = None
            self.scan = self.odom = None
            self.scan_received = self.odom_received = None
            self.map_received = False
        self.last_tick = now
        if not self.data_is_fresh(now):
            self.publish_result(self.navigator.waiting('missing_or_stale_sensor_data'))
            return
        if self.map_error:
            self.publish_result(self.navigator.waiting(self.map_error))
            return
        if self.require_map and not self.map_received:
            self.publish_result(self.navigator.waiting('missing_map'))
            return
        try:
            if self.odom.header.frame_id != self.odom_frame:
                raise ValueError('unexpected odometry frame')
            if self.odom.child_frame_id != self.base_frame:
                raise ValueError('odometry child frame must match base_frame')
            p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
            pose = Pose2D(p.x, p.y, quaternion_yaw(q.x, q.y, q.z, q.w))
            scan_transform = self.scan_transform()
            scan = ScanData(tuple(self.scan.ranges), self.scan.angle_min, self.scan.angle_max,
                            self.scan.angle_increment, self.scan.range_min, self.scan.range_max,
                            scan_transform)
            frame, final_goal = self.resolve_goal()
            goal_to_odom = self.lookup(
                self.odom_frame, frame, Time.from_msg(self.odom.header.stamp))
            final_odom = transform_point(final_goal, goal_to_odom)
            map_pose = map_goal = map_to_odom = None
            if self.mode == 'vfh_astar':
                map_to_odom = goal_to_odom
                odom_to_map = self.lookup(self.map_frame, self.odom_frame,
                                          Time.from_msg(self.odom.header.stamp))
                x, y = transform_point((pose.x, pose.y), odom_to_map)
                map_pose = Pose2D(x, y, wrap_angle(pose.yaw+odom_to_map.yaw))
                map_goal = final_goal
            result = self.navigator.step(scan, pose, self.odom.twist.twist.angular.z,
                                         final_odom, dt, now, map_pose, map_goal, map_to_odom)
        except (TransformException, ValueError) as error:
            result = self.navigator.waiting(f'missing_tf_or_invalid_input: {error}')
        self.publish_result(result)

    def destroy_node(self):
        """Join the planner worker and release ROS entities."""
        self.navigator.close()
        return super().destroy_node()


def main(args=None):
    """Run the ROS adapter and always issue a shutdown stop."""
    rclpy.init(args=args)
    node = VFHNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
