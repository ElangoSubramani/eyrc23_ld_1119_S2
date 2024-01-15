#!/usr/bin/env python3

"""
Controller for the drone
"""

# standard imports

# Copy has two main functions copy() -> same instance and deepcopy() -> new instance
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool
from std_msgs.msg import Float64


# Constants for ROLL, PITCH and THROTTLE min,max,base and sum_error limits.
# Limits are changed as @doubt temp
MIN_ROLL = 1250
BASE_ROLL = 1483 #Fixed 1485 when roll increases moves left
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000

MIN_PITCH = 1250
BASE_PITCH = 1495 #Fixed 1495 when pitch increases moves down
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 10000

MIN_TROTTLE = 1250
BASE_TROTTLE = 1400
MAX_TROTTLE = 1800
SUM_ERROR_THROTTLE_LIMIT = 1000


DRONE_WHYCON_POSE = [[], [], []]


class DroneController():

    def __init__(self, node):
        self.node = node

        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"

        self.arming_service_client = self.node.create_client(
            CommandBool, service_endpoint)
        # Setpoints for x, y, z respectively
        self.set_points = [0, 0, 22]

        # Current Error for roll, pitch and throttle
        self.error = [0.0, 0.0, 0.0]
        # Integral error for roll,pitch and throttle
        self.integeral_error = [0.0, 0.0, 0.0]
        # Derivative error for roll,pitch and throttle
        self.derivative_error = [0.0, 0.0, 0.0]
        # Previous error for roll,pitch and throttle
        self.previous_error = [0.0, 0.0, 0.0]
        # Sum error for roll,pitch and throttle'
        self.sum_error = [0.0, 0.0, 0.0]

        # LS[0]=roll, ls[1]=pitch, ls[2]=throttle

        # PID Controll factors for roll, pitch and throttle

        self.Kp = [0 * 0.01, 0 * 0.01, 0 * 0.01]
        self.Ki = [0 * 0.01, 0 * 0.01, 0 * 0.01]
        self.Kd = [0 * 0.01, 0 * 0.01, 0 * 0.01]

        # self.Kp = [195 * 0.01, 249 * 0.01, 280 * 0.01]
        # self.Ki = [63 * 0.00002, 39 * 0.00002, 401 * 0.00001]
        # self.Kd = [693 * 0.01, 722 * 0.01, 1681 * 0.01]
        # Similarly add callbacks for other subscribers are in 1/30s

        # Whycon subscriber

        self.whycon_sub = node.create_subscription(
            PoseArray, "/whycon/poses", self.whycon_poses_callback, rclpy.qos.QoSProfile(depth=10))

        # Subscribe to roll
        self.pid_roll = node.create_subscription(
            PidTune, "/pid_tuning_roll", self.pid_tune_roll_callback, rclpy.qos.QoSProfile(depth=10))

        # Subscribe to pitch
        self.pid_pitch = node.create_subscription(
            PidTune, "/pid_tuning_pitch", self.pid_tune_pitch_callback, rclpy.qos.QoSProfile(depth=10))

        # Subscribe to throttle
        self.pid_alt = node.create_subscription(
            PidTune, "/pid_tuning_altitude", self.pid_tune_throttle_callback, rclpy.qos.QoSProfile(depth=10))

        # Create publisher for sending commands to drone

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command", rclpy.qos.QoSProfile(depth=10))

        # Create publisher for publishing errors for plotting in plotjuggler

        self.pid_error_pub = node.create_publisher(
            PIDError, "/luminosity_drone/pid_error", 1)
        self.timer = node.create_timer(1/30, self.pid)  # 1/30s timer
        # Mannually added publishers for plotting in plotjuggler
        self.zero_publisher = node.create_publisher(Float64, 'zero_topic', rclpy.qos.QoSProfile(depth=10))
        self.lower_bound_publisher = node.create_publisher(
            Float64, 'lower_bound_topic', rclpy.qos.QoSProfile(depth=10))
        self.upper_bound_publisher = node.create_publisher(
            Float64, 'upper_bound_topic', rclpy.qos.QoSProfile(depth=10))
        self.throttle_publisher = node.create_publisher(
            Float64, 'throttle_topic', rclpy.qos.QoSProfile(depth=1))

    def custom_callback(self):
        # Publish values to respective topics
        zero_msg = Float64()
        zero_msg.data = 0.0
        self.zero_publisher.publish(zero_msg)

        lower_bound_msg = Float64()
        lower_bound_msg.data = -5.0
        self.lower_bound_publisher.publish(lower_bound_msg)

        upper_bound_msg = Float64()
        upper_bound_msg.data = 5.0
        self.upper_bound_publisher.publish(upper_bound_msg)
        # self.throttle_publisher.publish(float(self.rc_message.rc_throttle))

    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock(
        ).now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg

    # Callback function for roll

    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.05
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

    # Callback function for pitch

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.05
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1

    # Callback function for throttle

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        self.Ki[2] = msg.ki * 0.1
        self.Kd[2] = msg.kd * 0.1

    def limit(self, input_value, max_value, min_value):
        if input_value > max_value:
            return max_value
        if input_value < min_value:
            return min_value
        else:
            return input_value

         # Creating timed call back

    def pid(self):          # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        try:
            # Why con pose array is a list of poses. We are using only the first pose.
            # Other wise it shows list out of bound exception
            if self.drone_whycon_pose_array.poses:
                self.error[0] = -(self.drone_whycon_pose_array.poses[0].position.x - \
                    self.set_points[0])
                self.error[1] = -(self.drone_whycon_pose_array.poses[0].position.y - \
                    self.set_points[1])
                self.error[2] = self.drone_whycon_pose_array.poses[0].position.z - \
                    self.set_points[2]
        # Catch the exception thrown by anything wrong happens in calculating error and print the error message

        except Exception as e:
            print("PID exception", e)

        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # TODO
         # 1.) Check with the whycon axis to verify the roll and pitch axis and -  or  +
        self.derivative_error[0] = self.error[0] - self.previous_error[0]
        self.derivative_error[1] = self.error[1] - self.previous_error[1]
        self.derivative_error[2] = self.error[2] - self.previous_error[2]
        # print(self.derivative_error[2])
        # print("ff",self.error[2],self.previous_error[2])
        self.sum_error[0] = self.sum_error[0] + self.error[0]
        self.sum_error[1] = self.sum_error[1] + self.error[1]
        self.sum_error[2] = self.sum_error[2] + self.error[2]

        # Check for sum_error limits and apply anti windup.
        self.sum_error[0] = self.limit(
            self.sum_error[0], SUM_ERROR_ROLL_LIMIT, -SUM_ERROR_ROLL_LIMIT)
        self.sum_error[1] = self.limit(
            self.sum_error[1], SUM_ERROR_PITCH_LIMIT, -SUM_ERROR_PITCH_LIMIT)
        self.sum_error[2] = self.limit(
            self.sum_error[2], SUM_ERROR_THROTTLE_LIMIT, -SUM_ERROR_THROTTLE_LIMIT)
        # print("1",self.previous_error[2])

        # Save current error in previous error
        self.previous_error[2] = self.error[2]
        # print("2",self.previous_error[2])

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis
        self.roll = BASE_ROLL+(self.Kp[0]*self.error[0]+self.Ki[0]
                               * self.sum_error[0]+self.Kd[0]*self.derivative_error[0])
        self.pitch = BASE_PITCH+(self.Kp[1]*self.error[1]+self.Ki[1]
                                 * self.sum_error[1]+self.Kd[1]*self.derivative_error[1])
        self.throttle = BASE_TROTTLE + (self.Kp[2]*self.error[2]+self.Ki[2]
                                        * self.sum_error[2]+self.Kd[2]*self.derivative_error[2])
        # print(self.Kd[2]*self .derivative_error[2])

        # This will call the custom_callback function to publish values for plotting in plotjuggler
        self.custom_callback()
        # TODO
        # 1. Try Limiting the valutes  x,y,z before sending to the lowpass filter

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis

        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch
       
    # ------------------------------------------------------------------------------------------------------------------------
             
        self.throttle=self.limit(self.throttle,MAX_TROTTLE,MIN_TROTTLE)
        self.pitch=self.limit(self.pitch,MAX_PITCH,MIN_PITCH)
        self.roll=self.limit(self.roll,MAX_ROLL,MIN_ROLL)

        self.publish_data_to_rpi(self.roll, self.pitch, self.throttle) 
      
        # self.publish_data_to_rpi(self.roll, self.pitch, self.throttle)

        # Replace the roll pitch and throttle values as calculated by PID

        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=float(self.rc_message.rc_throttle),
                zero_error=float(self.throttle),
            )
        )

    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)

        # BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([roll, pitch, throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return "Not enough data"
            order = 3
            fs = 60
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        self.rc_message.rc_throttle = self.limit(
            self.rc_message.rc_throttle, MAX_TROTTLE, MIN_TROTTLE)
        self.rc_message.rc_pitch = self.limit(
            self.rc_message.rc_pitch, MAX_PITCH, MIN_PITCH)
        self.rc_message.rc_roll = self.limit(
            self.rc_message.rc_roll, MAX_ROLL, MIN_ROLL)

        # if self.rc_message.rc_roll > MAX_ROLL:  # checking range i.e. bet 1000 and 2000
        #     self.rc_message.rc_roll = MAX_ROLL
        # elif self.rc_message.rc_roll < MIN_ROLL:
        #     self.rc_message.rc_roll = MIN_ROLL

        # Similarly add bounds for pitch yaw and throttle
       
        # print(self.rc_message.rc_throttle)

        self.rc_pub.publish(self.rc_message)

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C.
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone

    def disarm(self):
        self.node.get_logger().info("Calling Disarm service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()  # cTemporarily commented for testing
    time.sleep(6)
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            # print(controller.last_whycon_pose_received_at)
            if (node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at) > 1:

                node.get_logger().error("Unable to detect WHYCON poses")
            # Sleep for 1/30 secs, It will give 0.033 Secs to complete the single spin (Callbacks..)
            rclpy.spin(node)  # Spin_once  changed to spin the node
            """rclpy.spin_once is consistently running at approximately 40 Hz instead of the expected 30 Hz, 
            it suggests that the code inside your loop is likely taking less than 0.033 seconds to execute, 
            allowing the loop to complete more iterations within the desired time period."""
            # time.sleep(0.033)

    except Exception as err:
        print("main function exception", (err))

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
