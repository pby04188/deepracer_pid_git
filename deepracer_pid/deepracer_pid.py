import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import String
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from geometry_msgs.msg import Twist, Vector3
from math import pi, radians
from path_following.lib.utils import pidController
import time
import sys, signal

class DeepracerPID(Node):

    def __init__(self):
        super().__init__('deepracer_pid')
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.configure()
        
        self.target_vel_sub = self.create_subscription(
            Twist,
            '/deepracer/cmd_vel',
            self.sub_target_vel,
            QOS_RKL10V
        )
        
        self.current_vel_sub = self.create_subscription(
            Twist,
            '/deepracer/current_vel',
            self.sub_current_vel,
            QOS_RKL10V
        )

        self.action_publisher = self.create_publisher(
            ServoCtrlMsg,
            '/ctrl_pkg/servo_msg',
            QOS_RKL10V
        )
        
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.pid_lin_vel = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        self.pid_ang_vel = pidController(self.p_gain, self.i_gain, self.d_gain, self.control_time)
        
    def configure(self):
        '''self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steer', 10)
        self.declare_parameter('vehicle_length', 0.17)
        self.declare_parameter('frequency', 20)
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('i_gain', 0.0)
        self.declare_parameter('d_gain', 0.05)
        self.declare_parameter('mode', 'keyboard')
        
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer_deg = self.get_parameter('max_steer').value
        self.max_steer = radians(self.max_steer_deg)
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.frequency = self.get_parameter("frequency").value
        self.control_time = float(1)/float(self.get_parameter("frequency").value)
        self.p_gain = self.get_parameter("p_gain").value
        self.i_gain = self.get_parameter("i_gain").value
        self.d_gain = self.get_parameter("d_gain").value'''
        
        self.mode = 'auto'
        self.max_speed = 1.0
        self.max_steer = 15
        self.p_gain = 1.0
        self.i_gain = 0.0
        self.d_gain = 0.0
        self.vehicle_length = 0.17
        self.control_time = 0.05
        
    def action_publish(self):
        #Function publishes the action and sends it to servo.
        #Args:
        #    target_steer (float): Angle value to be published to servo.
        #    target_speed (float): Throttle value to be published to servo.
        
        result = ServoCtrlMsg()
        result.angle, result.throttle = self.twist2Servo()
        self.get_logger().info(f"Publishing to servo: Steering {result.angle} | Throttle {result.throttle}")
        self.action_publisher.publish(result)

    def sub_current_vel(self, msg):
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        
    def sub_target_vel(self, msg):
    	self.target_linear_vel = msg.linear.x
    	self.target_angular_vel = msg.angular.z
            
    def twist2Servo(self):
        # steer [-1.0 , 1.0]
        # speed [-1.0 , 1.0]
        
        # pid 제어 해볼랐는데 경로가 이상해짐
        linear_vel = self.current_linear_vel
        ##angular_vel = self.current_angular_vel
        linear_vel += self.pid_lin_vel.pid(self.target_linear_vel, self.current_linear_vel)
        ##angular_vel += self.pid_ang_vel.pid(self.target_angular_vel, self.current_angular_vel)
        ##linear_vel = self.target_linear_vel
        ##angular_vel = self.target_angular_vel
            
        target_throttle = linear_vel / self.max_speed
        #target_steer = (self.vehicle_length * angular_vel / (linear_vel + 0.0000001)) / self.max_steer
        target_angle = self.target_angular_vel

        if target_angle > 0.17:
            target_angle = 1.0
        elif target_angle < -0.17:
            target_angle = -1.0

        if target_throttle > 0.8:
            target_throttle = 0.8
        elif target_throttle < -0.8:
            target_throttle = -0.8
        
        '''if target_steer > self.max_steer:
            target_steer = self.max_steer
        elif target_steer < -self.max_steer:
            target_steer = -self.max_steer'''
        
        # if self.target_linear_vel > self.max_speed:
        #     self.target_linear_vel = self.max_speed
        # elif self.target_linear_vel < -self.max_speed:
        #     self.target_linear_vel = -self.max_speed
            
        # target_throttle = self.target_linear_vel / self.max_speed
        # target_steer = (self.vehicle_length * self.target_angular_vel / (self.target_linear_vel + 0.0000001)) / self.max_steer
        
        # if target_steer > self.max_steer:
        #     target_steer = self.max_steer
        # elif target_steer < -self.max_steer:
        #     target_steer = -self.max_steer
        '''if target_throttle < 0.5 and target_throttle > 0.0:
            target_throttle = 0.5
        elif target_throttle < 0.0 and target_throttle > -0.5:
            target_throttle = -0.5'''
        return target_angle, target_throttle

    def exit(self, signum, frame):
        result = ServoCtrlMsg()
        result.angle = 0.0
        result.throttle = 0.0
        self.action_publisher.publish(result)
        sys.exit(1)

def main(args = None):
        rclpy.init(args=args)
        node = DeepracerPID()
        signal.signal(signal.SIGINT, node.exit)
        try:
            while rclpy.ok():
                rclpy.spin_once(node)
                node.action_publish()
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            

if __name__ == '__main__':
    main()

