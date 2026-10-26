import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def wrap_angle(angle):
    """Return an angle in the interval [-1.0472, 1.0472]."""
    #pull this value from LiDAR range in my_robot file (currently -120 to 120 degrees)
    return math.atan2(math.sin(angle), math.cos(angle))

class VFHNode(Node):

    def __init__(self):
        super().__init__('vfh_node')

        # ROS parameters - all based off physical robot, I so badly need a way to define with variables
        self.declare_parameter('goal_x', 4.0) #set really simple base goal
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.20)

        #this should remain semi-permanent since from measured values
        self.declare_parameter('robot_radius', 0.25) #conservative (.25m radius is about 10 inches)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('influence_distance', 1.50) #tune this
        self.declare_parameter('stop_distance', 0.45)

        self.declare_parameter('sector_width_deg', 5.0) #i assigned these randomly
        self.declare_parameter('histogram_threshold', 0.15)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.00)

        #eventually want to tune these with RL
        self.declare_parameter('goal_weight', 2.0)
        self.declare_parameter('heading_weight', 0.3)
        self.declare_parameter('smoothness_weight', 0.5)


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

        #histogram config
        requested_width = float(param('sector_width_deg'))
        self.num_sectors = max(8,round(120.0 / requested_width))
        self.sector_width = (2.0 * math.pi / self.num_sectors)
        self.sector_angles = (-math.pi + (np.arange(self.num_sectors) + 0.5) * self.sector_width)
        self.previous_direction = 0.0

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

    #Making polar histogram now
    def build_histogram(self, scan):
        density = np.zeros(self.num_sectors, dtype=float)

        observed = np.zeros(self.num_sectors, dtype=bool)

        ranges = np.asarray(scan.ranges,dtype=float)

        bearings = (scan.angle_min + np.arange(len(ranges)) * scan.angle_increment)

        inflated_radius = (self.robot_radius + self.safety_margin)

        for distance, bearing in zip(ranges, bearings):
            # A NaN reading is unknown.
            # Positive infinity denotes no return within the sensor's measurement range.
            if np.isnan(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                if not np.isposinf(distance):
                    continue

            bearing = wrap_angle(float(bearing))

            # Mark this direction as observed - def an easier way to do this
            index = int(math.floor((bearing + math.pi) / self.sector_width)) % self.num_sectors
            observed[index] = True

            # An infinite return is treated as clear within the LiDAR's maximum range
            if np.isposinf(distance):
                continue

            if distance >= self.influence_distance:
                continue

            # Inflate obstacles to account for the robot's physical footprint.
            inflation_angle = math.asin(min(1.0, inflated_radius / max(distance,1e-6)))

            # Distance-weighted obstacle density, again check equation in thesis notes
            weight = (1.0 - distance / self.influence_distance) ** 2
            angular_error = np.abs(np.arctan2(np.sin(self.sector_angles - bearing), np.cos(self.sector_angles - bearing)))
            blocked = (angular_error <= inflation_angle + self.sector_width / 2.0)
            density[blocked] = np.maximum(density[blocked],weight)

        return density, observed

    #Selecting the direction
    def choose_direction(self,density,observed,goal_angle):
        # Only consider angular sectors for which the LiDAR supplied measurements.
        clear = ((density < self.histogram_threshold) & observed)

        #currently im having controller only driver forward and rotate
        #no reversal or side way movement, yet

        clear = clear & (np.abs(self.sector_angles) < 2.7)
        candidates = self.sector_angles[clear]

        if len(candidates) == 0:
            return None

        costs = []

        for angle in candidates: #look at notes again!
            goal_error = abs(wrap_angle(angle - goal_angle))
            heading_error = abs(angle)
            smoothness_error = abs(wrap_angle(angle - self.previous_direction))
            cost = (self.w_goal * goal_error + self.w_heading * heading_error+ self.w_smooth * smoothness_error)
            costs.append(cost)

        best_index = int(np.argmin(costs))
        best_angle = float(candidates[best_index])

        self.previous_direction = best_angle
        return best_angle

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

        selected = self.choose_direction(density,observed,goal_angle)

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
        cmd.angular.z = float( np.clip(1.2 * selected, -self.max_angular_speed, self.max_angular_speed))

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
