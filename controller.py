import rclpy                                   # ROS 2 Python library
import numpy as np                             # numpy for array maths
from rclpy.node import Node                    # base class for all ROS nodes
from geometry_msgs.msg import Twist            # velocity command message
from sensor_msgs.msg import LaserScan          # laser scan message


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')                                   # give the node a name
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)               # publisher for velocity commands
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)  # listen to the laser
        self.create_timer(0.1, self.callback_velocity)                        # run the control loop 10 times per second

        self.got_scan = False        # becomes True after the first scan arrives (prevents startup crash)
        self.target_angle = 0.0      # direction (radians) of the emptiest space; 0 = straight ahead
        self.front_min = 10.0        # closest obstacle in front (meters); starts large = "nothing close"
        self.turning = False     # True while the robot is turning away from an obstacle
        self.turn_dir = 1.0      # +1 = turn left, -1 = turn right (held while turning)
    def callback_scan(self, msg):
        self.angle_min = msg.angle_min                    # angle of the first laser reading
        self.angle_incr = msg.angle_increment             # angle between two readings
        ranges = np.array(msg.ranges)                     # convert the list of distances to a numpy array
        cleaned = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max)  # replace nan/inf with max range

        # --- find the emptiest direction ---
        il, ih = self.angle_slicer(np.deg2rad(120))       # index range for -120 deg to +120 deg (front area only)
        sliced = cleaned[il:ih + 1]                       # keep only the front area readings
        smooth = np.convolve(sliced, np.ones(80) / 80, mode='same')  # average each reading with its neighbors (~16 deg)
        best_index = np.argmax(smooth) + il               # position of the largest average, converted to a real scan index
        self.target_angle = self.angle_min + best_index * self.angle_incr  # convert that index into an angle

        # --- safety: closest obstacle straight ahead ---
        fl, fh = self.angle_slicer(np.deg2rad(30))        # index range for -30 deg to +30 deg
        self.front_min = cleaned[fl:fh + 1].min()         # smallest distance in that cone

        self.got_scan = True                              # we now have valid data

        # print once per second to help debugging
        self.get_logger().info(
            f"target {np.rad2deg(self.target_angle):.0f} deg, front_min {self.front_min:.2f} m",
            throttle_duration_sec=1.0)

    def callback_velocity(self):
        if not self.got_scan:                       # no scan yet, do nothing
            return
        vel = Twist()                               # empty velocity command

        if not self.turning and self.front_min < 0.6:   # something close ahead, start a turn
            self.turning = True                     # remember that we are turning
            # pick the turn direction ONCE, from the side with the emptiest space
            self.turn_dir = 1.0 if self.target_angle >= 0 else -1.0

        if self.turning:
            vel.linear.x = 0.0                      # stop driving forward
            vel.angular.z = 0.4 * self.turn_dir     # keep turning the same way
            if self.front_min > 1.0:                # front is clearly open now (bigger than 0.6, so no flicker)
                self.turning = False                # stop turning, go straight again
        else:
            vel.linear.x = 0.15                     # nothing ahead: just drive straight
            vel.angular.z = 0.0                     # no steering

        self.pub.publish(vel)                       # send the command

    def angle_slicer(self, angle):
        index_low = int((-angle - self.angle_min) / self.angle_incr)   # index of -angle
        index_high = int((angle - self.angle_min) / self.angle_incr)   # index of +angle
        return index_low, index_high                                   # return the slice limits


def main():
    rclpy.init()                  # start ROS
    node = Controller()           # create the node
    rclpy.spin(node)              # keep it running and handle callbacks
    node.destroy_node()           # cleanup
    rclpy.shutdown()              # stop ROS


if __name__ == "__main__":
    main()