import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def wrap_angle(angle):
    """Wrap an angular difference to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

class VFHNode(Node):

    def __init__(self):
        super().__init__('vfh_node')

        # ROS parameters - all based off physical robot, I so badly need a way to define with variables
        self.declare_parameter('goal_x', 10.0)  # default; YAML overrides
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.20)

        #this should remain semi-permanent since from measured values
        self.declare_parameter('robot_radius', 0.25) #conservative (.25m radius is about 10 inches)
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('influence_distance', 1.50) #tune this
        self.declare_parameter('stop_distance', 0.45)

        self.declare_parameter('sector_width_deg', 5.0) #i assigned these randomly
        self.declare_parameter('histogram_threshold', 0.15)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.00)

        #eventually want to tune these with RL
        self.declare_parameter('goal_weight', 3.00)
        self.declare_parameter('heading_weight', .5)
        self.declare_parameter('smoothness_weight', .5)

        # Hysteresis and damped turning (initial tuning values).
        self.declare_parameter('switch_hysteresis', 0.35)
        self.declare_parameter('turn_side_penalty', 1.2)
        self.declare_parameter('angular_kp', 0.8)
        self.declare_parameter('angular_kd', 0.25)
        self.declare_parameter('angular_accel_limit', 0.8)  # rad/s^2



        #load parameters above
        def param(name):
            return self.get_parameter(name).value

        #these are from equations in thesis notes doc
        self.goal_x = float(param('goal_x'))
        self.goal_y = float(param('goal_y'))
        self.goal_tolerance = float(param('goal_tolerance'))
        self.robot_radius = float(param('robot_radius'))
        self.safety_margin = float(param('safety_margin'))
        self.influence_distance = float(param('influence_distance'))
        self.stop_distance = float(param('stop_distance'))
        self.max_linear_speed = float(param('max_linear_speed'))
        self.max_angular_speed = float(param('max_angular_speed'))
        self.histogram_threshold = float(param('histogram_threshold'))
        self.w_goal = float(param('goal_weight'))
        self.w_heading = float(param('heading_weight'))
        self.w_smooth = float(param('smoothness_weight'))
        self.switch_hysteresis = float(param('switch_hysteresis'))
        self.turn_side_penalty = float(param('turn_side_penalty'))
        self.angular_kp = float(param('angular_kp'))
        self.angular_kd = float(param('angular_kd'))
        self.angular_accel_limit = float(param('angular_accel_limit'))


        # Use 5-degree bins across the *observed* 120-degree field of view.
        # A 120-degree scan DOES NOT mean that the other 240 degrees are clear.
        requested_width = float(param('sector_width_deg'))
        if requested_width <= 0.0:
            raise ValueError('sector_width_deg must be positive')
        self.requested_width = math.radians(requested_width)
        self.num_sectors = None
        self.sector_width = None
        self.sector_angles = None
        self.scan_angle_min = None
        self.scan_angle_max = None

        # Persist steering intent in the *odom/world frame*, not base_link.
        self.previous_world_heading = None
        self.avoidance_side = 0
        self.clear_goal_cycles = 0
        self.last_angular_command = 0.0

        # Robot state from publishers
        self.scan = None
        self.odom = None
        self.last_scan_time = None
        self.last_odom_time = None

        # ros subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) #might want to use wheel odom here instead
        #publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Run controller at 10 Hz, affects line below too
        self.timer = self.create_timer(0.1, self.control_loop) 

        self.get_logger().info('VFH navigation node started')

    #sensor calls
    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_time = (self.get_clock().now())
    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_time = (self.get_clock().now())

    #Random functions that are kinda useful
    def publish_stop(self):
        self.last_angular_command = 0.0
        self.cmd_pub.publish(Twist())

    def get_robot_pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        # Quaternion to planar yaw - remember ece414 drone project
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        return p.x, p.y, yaw

    def data_is_fresh(self):
        if self.scan is None or self.odom is None:
            return False

        if (self.last_scan_time is None or self.last_odom_time is None):
            return False

        #timestamp things
        now = self.get_clock().now()
        scan_age = (now - self.last_scan_time).nanoseconds / 1e9
        odom_age = (now - self.last_odom_time).nanoseconds / 1e9

        return (0.0 <= scan_age < 0.5 and 0.0 <= odom_age < 0.5)

    def update_scan_geometry(self, scan):
        # Derive the angular histogram from the real LaserScan limits.
        # With [-60,+60] degrees and 5-degree requested sectors: 24 bins.
        if (self.num_sectors is not None and
                self.scan_angle_min == scan.angle_min and
                self.scan_angle_max == scan.angle_max):
            return

        fov = float(scan.angle_max - scan.angle_min)
        if fov <= 0.0:
            raise ValueError('LiDAR field of view must be positive')
        self.num_sectors = max(1, int(round(fov / self.requested_width)))
        self.sector_width = fov / self.num_sectors
        self.sector_angles = (
            float(scan.angle_min) +
            (np.arange(self.num_sectors) + 0.5) * self.sector_width
        )
        self.scan_angle_min = scan.angle_min
        self.scan_angle_max = scan.angle_max

    def build_histogram(self, scan):
        self.update_scan_geometry(scan)
        density = np.zeros(self.num_sectors, dtype=float)
        valid_counts = np.zeros(self.num_sectors, dtype=int)
        total_counts = np.zeros(self.num_sectors, dtype=int)

        ranges = np.asarray(scan.ranges, dtype=float)
        bearings = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        inflated_radius = self.robot_radius + self.safety_margin

        for distance, bearing in zip(ranges, bearings):
            idx = int(np.clip(
                math.floor((bearing - scan.angle_min) / self.sector_width),
                0, self.num_sectors - 1
            ))
            total_counts[idx] += 1

            # NaN / out-of-range data are UNKNOWN, not evidence of clear space.
            valid = (np.isposinf(distance) or
                     (np.isfinite(distance) and
                      scan.range_min <= distance <= scan.range_max))
            if not valid:
                continue
            valid_counts[idx] += 1
            if np.isposinf(distance) or distance >= self.influence_distance:
                continue

            inflation_angle = math.asin(min(1.0, inflated_radius / max(distance, 1e-6)))
            weight = (1.0 - distance / self.influence_distance) ** 2
            angular_error = np.abs(np.arctan2(
                np.sin(self.sector_angles - bearing),
                np.cos(self.sector_angles - bearing)
            ))
            blocked = angular_error <= inflation_angle + self.sector_width / 2.0
            density[blocked] = np.maximum(density[blocked], weight)

        # Require at least 80% of a sector's rays to be usable before
        # considering that sector observed. Adjust if sensor has dropouts.
        observed = ((total_counts > 0) &
                    (valid_counts >= 0.80 * total_counts))
        return density, observed

    def choose_direction(self, density, observed, goal_angle, yaw):
        clear = (density < self.histogram_threshold) & observed
        candidates = self.sector_angles[clear]
        if len(candidates) == 0:
            return None

        if self.previous_world_heading is None:
            previous_relative = None
        else:
            previous_relative = wrap_angle(self.previous_world_heading - yaw)

        # Check whether the direct goal direction is currently verified clear.
        # A goal outside the 120-degree scan is unknown, NOT clear.
        goal_in_fov = (
            self.sector_angles[0] <= goal_angle <= self.sector_angles[-1]
        )
        goal_clear = False
        if goal_in_fov:
            idx = int(np.argmin(np.abs(self.sector_angles - goal_angle)))
            goal_clear = bool(clear[idx])

        if goal_clear and abs(goal_angle) < math.radians(20):
            self.clear_goal_cycles += 1
        else:
            self.clear_goal_cycles = 0
        if self.clear_goal_cycles >= 10:
            self.avoidance_side = 0

        costs = np.empty(len(candidates), dtype=float)
        for i, angle in enumerate(candidates):
            goal_error = abs(wrap_angle(angle - goal_angle))
            heading_error = abs(angle)
            smoothness_error = (0.0 if previous_relative is None else
                                abs(wrap_angle(angle - previous_relative)))
            cost = (self.w_goal * goal_error +
                    self.w_heading * heading_error +
                    self.w_smooth * smoothness_error)

            # When an obstacle requires steering around it, discourage
            # alternating between left and right every scan. This is a
            # penalty, not an absolute ban: safer alternatives remain possible.
            if self.avoidance_side and np.sign(angle) != self.avoidance_side:
                cost += self.turn_side_penalty
            costs[i] = cost

        best = int(np.argmin(costs))

        # Preserve the current world-frame steering direction unless another
        # direction is meaningfully better (hysteresis).
        if previous_relative is not None:
            differences = np.abs(np.arctan2(
                np.sin(candidates - previous_relative),
                np.cos(candidates - previous_relative)
            ))
            old = int(np.argmin(differences))
            if (differences[old] <= 1.5 * self.sector_width and
                    costs[old] <= costs[best] + self.switch_hysteresis):
                best = old

        selected = float(candidates[best])
        self.previous_world_heading = wrap_angle(yaw + selected)

        if not goal_clear and self.avoidance_side == 0 and abs(selected) > math.radians(15):
            self.avoidance_side = 1 if selected > 0 else -1

        return selected

    #main nav control loop

    def control_loop(self):
        # Stop on missing or stale data.
        if not self.data_is_fresh():
            self.publish_stop()
            return

        x, y, yaw = self.get_robot_pose()

        # Goal is expressed in the odom frame.
        dx = self.goal_x - x
        dy = self.goal_y - y

        distance_to_goal = math.hypot(dx,dy)

        if distance_to_goal < self.goal_tolerance:
            self.publish_stop()
            return

        # Convert the world-relative goal into a robot-relative direction.
        goal_angle = wrap_angle(math.atan2(dy, dx) - yaw)

        density, observed = self.build_histogram(self.scan)

        selected = self.choose_direction(density,observed,goal_angle,yaw)

        if selected is None:
            # No sufficiently clear direction - this is where I'll define recovering behavior
            self.publish_stop()
            return

        # forward collsion checkS
        ranges = np.asarray(self.scan.ranges,dtype=float)
        bearings = (self.scan.angle_min+ np.arange(len(ranges))* self.scan.angle_increment)

        #full inflated robot width to define a conservative forward arc.
        forward_half_angle = math.atan2(self.robot_radius + self.safety_margin, max(self.stop_distance, 0.01))
        forward_mask = (np.abs(bearings) <= forward_half_angle)
        forward_ranges = ranges[forward_mask]
        finite_ranges = forward_ranges[np.isfinite(forward_ranges) & (forward_ranges >= self.scan.range_min) & (forward_ranges <= self.scan.range_max)]

        # Unknown forward observations do not authorize forward motion. what is this doing...
        forward_observed = observed[np.abs(self.sector_angles) <= forward_half_angle].all()

        if not forward_observed:
            self.publish_stop()
            return

        if len(finite_ranges) > 0:
            front_distance = float(np.min(finite_ranges))
        else:
            front_distance = float('inf')

        # put into cmd_vel
        cmd = Twist()
        # P + yaw-rate damping; then limit change in commanded angular speed.
        yaw_rate = float(self.odom.twist.twist.angular.z)
        target_omega = float(np.clip(
            self.angular_kp * selected - self.angular_kd * yaw_rate,
            -self.max_angular_speed, self.max_angular_speed
        ))
        if (abs(selected) < math.radians(5.0) and
                abs(goal_angle) < math.radians(5.0)):
            target_omega = 0.0  # avoid chattering across the central bins
        dt = 0.1  # timer period (sim seconds)
        delta_limit = self.angular_accel_limit * dt
        cmd.angular.z = float(self.last_angular_command + np.clip(
            target_omega - self.last_angular_command,
            -delta_limit, delta_limit
        ))
        self.last_angular_command = cmd.angular.z

        # Move forward only when facing
        # approximately along the selected
        # collision-free direction.

        if abs(selected) < 0.45:
            cmd.linear.x = ( self.max_linear_speed * max( 0.0, 1.0 - abs(selected) / 0.45) )

        else:
            # Rotate in place toward the
            # selected direction.
            cmd.linear.x = 0.0

        # Emergency stop for nearby obstacles.
        if front_distance < self.stop_distance:
            cmd.linear.x = 0.0

        # Stop if any invalid measurement is present in the forward arc.
        if np.any(~np.isfinite(forward_ranges) & ~np.isposinf(forward_ranges)):
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VFHNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
