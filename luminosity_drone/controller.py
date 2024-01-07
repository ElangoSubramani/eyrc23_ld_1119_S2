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


# Constants for ROLL, PITCH and THROTTLE min,max,base and sum_error limits.

MIN_ROLL = 1250
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000

MIN_PITCH = 1250
BASE_PITCH = 1500
MAX_PITCH = 1600
SUM_ERROR_PITCH_LIMIT = 10000

MIN_TROTTLE = 1250
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_THROTTLE_LIMIT = 10000


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
        self.set_points = [0, 0, 0]

        # Current Error for roll, pitch and throttle
        self.error = [0, 0, 0]
        # Integral error for roll,pitch and throttle
        self.integeral_error = [0, 0, 0]
        # Derivative error for roll,pitch and throttle
        self.derivative_error = [0, 0, 0]
        # Previous error for roll,pitch and throttle
        self.previous_error = [0, 0, 0]
        # Previous error for roll,pitch and throttle'
        self.sum_error = [0, 0, 0]

        # LS[0]=roll, ls[1]=pitch, ls[2]=throttle

        # PID Controll factors for roll, pitch and throttle

        self.Kp = [0 * 0.01, 0 * 0.01, 0 * 0.01]
        self.Ki = [0 * 0.01, 0 * 0.01, 0 * 0.01]
        self.Kd = [0 * 0.01, 0 * 0.01, 0 * 0.01]
        # Similarly add callbacks for other subscribers
        # TODO
        # 1. Create pubs and subs based on ros2 topic
        # 2. Complete PID Algorithm
        # 3. Test the algorithm mannually

        # Create subscriber for WhyCon

        self.whycon_sub = node.create_subscription(
            PoseArray, "/whycon/poses", self.whycon_poses_callback, 1)

        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required

        self.pid_alt = node.create_subscription(
            PidTune, "/pid_tuning_altitude", self.pid_tune_throttle_callback, 1)

        # Create publisher for sending commands to drone

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command", 1)

        # Create publisher for publishing errors for plotting in plotjuggler

        self.pid_error_pub = node.create_publisher(
            PIDError, "/luminosity_drone/pid_error", 1)

    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock(
        ).now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    def pid(self):          # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        try:
            self.error[0] = self.drone_whycon_pose_array.poses[0].position.x - \
                self.set_points[0]
            self.error[1] = self.drone_whycon_pose_array.poses[1].position.y - \
                self.set_points[1]
            self.error[2] = self.drone_whycon_pose_array.poses[2].position.z - \
                self.set_points[2]
        # Similarly calculate error for y and z axes

        except:
            pass

        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # self.integral[0] = (self.integral[0] + self.error[0])
        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        # Save current error in previous error

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis

        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch

    # ------------------------------------------------------------------------------------------------------------------------

        # Replaced 1000 by 1450 as instruced in the video
        self.publish_data_to_rpi(roll=1500, pitch=1500, throttle=1450)

        # Replace the roll pitch and throttle values as calculated by PID

        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.0,
                zero_error=0.0,
            )
        )

    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)

        # BUTTERWORTH FILTER
        # span = 15
        # for index, val in enumerate([roll, pitch, throttle]):
        #     DRONE_WHYCON_POSE[index].append(val)
        #     if len(DRONE_WHYCON_POSE[index]) == span:
        #         DRONE_WHYCON_POSE[index].pop(0)
        #     if len(DRONE_WHYCON_POSE[index]) != span-1:
        #         return
        #     order = 3
        #     fs = 60
        #     fc = 5
        #     nyq = 0.5 * fs
        #     wc = fc / nyq
        #     b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
        #     filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
        #     if index == 0:
        #         self.rc_message.rc_roll = int(filtered_signal[-1])
        #     elif index == 1:
        #         self.rc_message.rc_pitch = int(filtered_signal[-1])
        #     elif index == 2:
        #         self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:  # checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL

        # Similarly add bounds for pitch yaw and throttle

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
    controller.arm()
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 1:
                node.get_logger().error("Unable to detect WHYCON poses")
            # Sleep for 1/30 secs, It will give 0.033 Secs to complete the single spin (Callbacks..)
            rclpy.spin_once(node, timeout_sec=0.033)

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
