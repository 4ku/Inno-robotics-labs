import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib import pyplot as plt


class MinimalPublisher(Node):
    def __init__(self, Ts, duration=10):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(
            Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/hagen/odom", self.set_pose, 20)
        self.q_calculated = None
        self.q_actual = None
        self.r = 0.04  # Wheel radius
        self.L = 0.08  # Distance between the wheels
        self.Ts = Ts  # Sampling time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.time_utilized = 0.0
        self.duration = duration

        # Collect data to plot it
        self.t = []
        self.Q_actual = []
        self.Q_calculated = []

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass

    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap) > np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def control(self):
        if (self.q_calculated is None):
            return

        if (self.duration <= self.time_utilized):
            print("End of simulation")
            self.send_vel(0.0, 0.0)
            self.end_controller = True
            return

        def case1():
            v = 0.5
            w = 0.0
            return v, w

        def case2():
            v = 1.0
            w = 2.0
            return v, w

        def case3():
            v = 0.0
            w = 2.0
            return v, w

        def case4():
            wL = 20  # Left wheel velocity
            wR = 18  # Right wheel velocity
            v = self.r/2*(wR+wL)  # Robot velocity
            w = self.r/self.L*(wR-wL)  # Robot angular velocity
            return v, w

        # Use necessary case
        v, w = case4()

        # Integration
        dq = np.array([v*np.cos(self.q_calculated[2]),
                      v*np.sin(self.q_calculated[2]), 2*w])
        # Trapezoidal numerical integration
        dq = np.array([v*np.cos(self.q_calculated[2]+self.Ts*w/2),
                       v*np.sin(self.q_calculated[2]+self.Ts*w/2), 2*w])
        self.q_calculated = self.q_calculated + self.Ts*dq
        # Map orientation angle to [-pi, pi]
        self.q_calculated[2] = self.wrap_to_pi(self.q_calculated[2])

        self.time_utilized = self.time_utilized + self.Ts
        self.send_vel(v, w)

        self.t.append(self.time_utilized)
        self.Q_calculated.append(self.q_calculated)
        self.Q_actual.append(self.q_actual)

    def timer_callback(self, ):
        if not self.end_controller:
            self.control()
        else:
            self.destroy_timer(self.timer)
            return

    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.q_actual = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        if (self.q_calculated is None):
            self.q_calculated = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(Ts=0.033, duration=5)
    while not minimal_publisher.end_controller and rclpy.ok():
        try:
            rclpy.spin_once(minimal_publisher)
        except KeyboardInterrupt:
            break
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    # Plot data
    t = minimal_publisher.t
    Q_actual = np.array(minimal_publisher.Q_actual)
    Q_calculated = np.array(minimal_publisher.Q_calculated)

    fig = plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(t, Q_actual[:, 0], label="Actual X")
    plt.plot(t, Q_calculated[:, 0], label="Calculated X")
    plt.xlabel("time t")
    plt.ylabel("X coord")
    plt.title("X coord")
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(t, Q_actual[:, 1], label="Actual Y")
    plt.plot(t, Q_calculated[:, 1], label="Calculated Y")
    plt.xlabel("time t")
    plt.ylabel("Y coord")
    plt.title("Y coord")
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(t, Q_actual[:, 2], label="Actual Phi")
    plt.plot(t, Q_calculated[:, 2], label="Calculated Phi")
    plt.xlabel("time t")
    plt.ylabel("Phi angle")
    plt.title("Phi angle")
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(Q_actual[:, 0], Q_actual[:, 1], label="Actual X-Y")
    plt.plot(Q_calculated[:, 0], Q_calculated[:, 1], label="Calculated X-Y")
    plt.xlabel("X coord")
    plt.ylabel("Y coord")
    plt.title("X-Y coords")
    plt.legend()

    fig.suptitle("${W_L}$=20, ${W_R}$=18", fontsize=16)
    plt.tight_layout()
    plt.savefig("Case_4.png")
    plt.show()


if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
