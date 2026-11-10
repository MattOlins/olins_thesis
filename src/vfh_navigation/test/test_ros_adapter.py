"""ROS adapter tests using actual Jazzy messages, clocks, and TF2 buffers."""

import math

import pytest

try:
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    import rclpy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from vfh_navigation.vfh_node import VFHNode
except ModuleNotFoundError:
    pytest.skip('ROS Python modules unavailable', allow_module_level=True)


class Capture:
    """Capture adapter outputs while exercising actual ROS input types."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    instance = VFHNode()
    for name in ('cmd_pub', 'state_pub', 'path_pub', 'target_pub'):
        setattr(instance, name, Capture())
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def prime(node, mounting=True):
    """Supply current sensor data and optional mounting TF."""
    stamp = node.get_clock().now().to_msg()
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'lidar', stamp
    scan.angle_min, scan.angle_max = -math.pi/3, math.pi/3
    scan.angle_increment = 2*math.pi/3/719
    scan.range_min, scan.range_max = .1, 10.
    scan.ranges = [math.inf]*720
    odom = Odometry()
    odom.header.frame_id, odom.header.stamp = 'odom', stamp
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.orientation.w = 1.
    node.scan_callback(scan)
    node.odom_callback(odom)
    if mounting:
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'lidar'
        transform.transform.translation.x = .111125
        transform.transform.rotation.w = 1.
        node.tf_buffer.set_transform_static(transform, 'test')
        odom_transform = TransformStamped()
        odom_transform.header.frame_id = 'odom'
        odom_transform.child_frame_id = 'base_footprint'
        odom_transform.transform.rotation.w = 1.
        node.tf_buffer.set_transform_static(odom_transform, 'test')


def diagnostic(node):
    return {entry.key: entry.value for entry in node.state_pub.messages[-1].status[0].values}


def test_missing_inputs_publish_zero_and_reason(node):
    node.control_loop()
    assert diagnostic(node)['state'] == 'WAITING'
    assert node.cmd_pub.messages[-1].linear.x == 0


def test_mounting_tf_is_required(node):
    prime(node, mounting=False)
    node.control_loop()
    assert diagnostic(node)['state'] == 'WAITING'
    assert 'missing_tf' in diagnostic(node)['reason']
    assert node.cmd_pub.messages[-1].linear.x == 0


def test_valid_ros_inputs_publish_motion_and_diagnostics(node):
    prime(node)
    node.control_loop()
    assert diagnostic(node)['state'] == 'TRACKING'
    assert node.cmd_pub.messages[-1].linear.x > 0
    assert node.target_pub.messages[-1].header.frame_id == 'odom'


def test_stale_acquisition_stops_even_if_receipt_is_new(node):
    prime(node)
    node.scan.header.stamp.sec -= 1
    node.control_loop()
    assert diagnostic(node)['state'] == 'WAITING'
    assert node.cmd_pub.messages[-1].linear.x == 0


def test_goal_replacement_resets_state_and_completes_final_goal(node):
    prime(node)
    node.control_loop()
    revision = node.navigator.goal_revision
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = .1
    node.goal_callback(goal)
    assert node.navigator.goal_revision > revision
    assert node.navigator.controller.selector.previous_world_heading is None
    node.control_loop()
    assert diagnostic(node)['state'] == 'SUCCEEDED'
    assert node.cmd_pub.messages[-1].linear.x == 0


def test_clock_reset_discards_old_sensor_state(node):
    prime(node)
    node.last_tick = node.get_clock().now().nanoseconds/1e9+100
    revision = node.navigator.goal_revision
    node.control_loop()
    assert node.navigator.goal_revision > revision
    assert node.scan is None and node.odom is None
    assert node.cmd_pub.messages[-1].linear.x == 0


def test_scan_projection_compensates_for_acquisition_delay(node):
    prime(node)
    now = Time.from_msg(node.odom.header.stamp)
    past = Time(nanoseconds=now.nanoseconds-100_000_000, clock_type=now.clock_type)
    for stamp, x in ((past, 0.0), (now, .1)):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_footprint'
        transform.header.stamp = stamp.to_msg()
        transform.transform.translation.x = x
        transform.transform.rotation.w = 1.0
        node.tf_buffer.set_transform(transform, 'test')
    node.odom.pose.pose.position.x = .1
    node.scan.header.stamp = past.to_msg()
    assert node.scan_transform().x == pytest.approx(.011125)


def test_comparison_readiness_does_not_expose_map_geometry_to_vfh(node):
    prime(node)
    node.require_map = True
    node.control_loop()
    assert diagnostic(node)['reason'] == 'missing_map'
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.width = message.info.height = 10
    message.info.resolution = .1
    message.data = [0]*100
    node.map_callback(message)
    node.control_loop()
    assert diagnostic(node)['state'] == 'TRACKING'
    assert node.navigator.grid is None
