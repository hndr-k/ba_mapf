# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Robotino3 driver."""

import sys
import math
from math import cos, dist, sin
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, TransformStamped, Pose
from tf2_ros import TransformBroadcaster
from rclpy.action import ActionClient
#from mps_control_actions.action import Conveyor, PickPlace, Deliver, Light, Reset, Base
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.utils import get_node_name_from_args
import _thread
from controller import Robot, Supervisor
from controller import PositionSensor, Node , Camera, DistanceSensor

class RobotinoDriver(WebotsNode):
    def __init__(self,
                 name,
                 args,
                 wheel_distance=0.1826,
                 wheel_radius=0.063,
                 wheel0 ='wheel0_joint', # back motor
                 wheel1 ='wheel1_joint', # front right
                 wheel2 ='wheel2_joint', # front left
                 command_topic='/cmd_vel',
                 odometry_frame='odom',
                 robot_base_frame='base_link'
                 ):
        super().__init__(name, args)
        self.start_device_manager({
            'robot': {'publish_base_footprint': False},
            'Hokuyo URG-04LX-UG01': {},
            'accelerometer+gyro': {'frame_id': 'imu_link'},
            'GPS': {'frame_id': 'gps_link'},
            'Webcam for Robotino 3' : {'frame_id' : 'camera_rotated'}
        })

        self._wheel_distance=0.1826
        self._wheel_radius=0.060
        self._body_radius= 0.175 #distance from wheel to center
        self._gear = 1.0
        self._v_x = 0.0
        self._omega = 0.0
        # Initialize motors
        self.motor0 = self.robot.getMotor(wheel0)
        self.motor1 = self.robot.getMotor(wheel1)
        self.motor2 = self.robot.getMotor(wheel2)
        n_devices = self.robot.getNumberOfDevices()
        self.get_logger().info('Number of Devices'+str(n_devices))
        index = 0
        while index < n_devices:
            self.get_logger().info("index: "+ str(index)+" "+ str(self.robot.getDeviceByIndex(index)))
            index += 1
        #Camera
        self.camera = self.robot.getDeviceByIndex(6)
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.camera.enableRecognitionSegmentation()

        #Init wheel motor and encoder
        self.motor0_posSensor = self.robot.getDeviceByIndex(1)
        self.motor1_posSensor = self.robot.getDeviceByIndex(3)
        self.motor2_posSensor = self.robot.getDeviceByIndex(5)
        self.motor0_posSensor.enable(self.timestep)
        self.motor1_posSensor.enable(self.timestep)
        self.motor2_posSensor.enable(self.timestep)
        
        self.motor0.setPosition(float('inf'))
        self.motor1.setPosition(float('inf'))
        self.motor2.setPosition(float('inf'))
        
        self.motor0.setVelocity(0.0)
        self.motor1.setVelocity(0.0)
        self.motor2.setVelocity(0.0)

        #Init gripper
        self.x_axes_motor = self.robot.getDevice("x_axes_motor")
        self.y_axes_motor = self.robot.getDevice("y_axes_motor")
        self.z_axes_motor = self.robot.getDevice("z_axes_motor")
        self.m_finger1 = self.robot.getDevice("motor_finger_1")
        self.m_finger2 = self.robot.getDevice("motor_finger_2")
        self.m_finger3 = self.robot.getDevice("motor_finger_3")

        self.x_axes_tgt = 0.0
        self.y_axes_tgt = 0.1
        self.z_axes_tgt = 0.2

        self.m_finger1.setPosition(float('inf'))
        self.m_finger2.setPosition(float('inf'))
        self.m_finger3.setPosition(float('inf'))

        self.m_finger1.setPosition(0.1)
        self.m_finger2.setPosition(0.1)
        self.m_finger3.setPosition(-0.1)

        self.m_finger1.setAvailableForce(4.5)
        self.m_finger2.setAvailableForce(2)
        self.m_finger3.setAvailableForce(2)

        #ActionClient to UPC UA Server
        #self._conveyor_ac = ActionClient(self, Conveyor, '/mps_control/M_DS_conveyor')

        self.create_subscription(Bool, 'gripper_state', self._gripper_state_cb, 1)
        self.create_subscription(Twist, 'cmd_vel', self._base_apply_speeds_cb, 1)
        self.create_subscription(Twist, 'cmd_vel/gripper_axes', self._gripper_apply_speeds_cb, 1)
        self.create_subscription(Imu, 'accelerometer', self._imu_cb, 1)
        self.create_subscription(Bool, 'mps_activ', self._start_mps_cb, 1)
        self._odometry_publisher = self.create_publisher(Odometry, 'odom', 1)
        self._mps_distance_pub = self.create_publisher(Float32, 'mps/euklid_distance', 1)
        self._mps_pose_pub = self.create_publisher(Pose, 'mps/pose', 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._imu_angle = 0.0
        self._imu_vel = 0.0

        self._prev_wheel0_ticks = 0.0
        self._prev_wheel1_ticks = 0.0
        self._prev_wheel2_ticks = 0.0

        self._prev_odom_x = 0.0
        self._prev_odom_y = 0.0
        self._prev_odom_omega = 0.0
        self._last_odometry_sample_time = self.robot.getTime()

    def _start_mps_cb(self, mps_state):
        self.get_logger().info('Start_mps_cb')
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Send Goal')
        #goal_msg = Conveyor.Goal()
        #goal_msg.mps = 'M-DS'
        #goal_msg.direction = 2
        #goal_msg.stop_pos = 2
        #self._conveyor_ac.wait_for_server()
        #self._send_goal_future = self._conveyor_ac.send_goal_async(goal_msg)
        #self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Actionserver finished')

    def goal_response_callback(self, future):
        goal_handle =  future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def _gripper_state_cb(self, gripper_open):
        self.m_finger1.setAvailableForce(8.5)
        self.m_finger2.setAvailableForce(4)
        self.m_finger3.setAvailableForce(4)
        self.get_logger().info('Gripper Triggered')
        if gripper_open.data:
            self.m_finger1.setPosition(0.3)
            self.m_finger2.setPosition(0.3)
            self.m_finger3.setPosition(-0.3)
        else:
            self.m_finger1.setPosition(0.0)
            self.m_finger2.setPosition(0.0)
            self.m_finger3.setPosition(0.0)

    def _imu_cb(self, imu):
        self._imu_angle = imu.angular_velocity.z
        self._imu_vel = math.sqrt(math.pow(imu.linear_acceleration.x,2) + math.pow(imu.linear_acceleration.y,2))

    def _gripper_apply_speeds_cb(self, twist):
        v_z = twist.linear.z
        if twist.linear.z > 0:
            self.z_axes_tgt += 0.001
            if self.z_axes_tgt > 0.2:
                self.z_axes_tgt = 0.2
            self.z_axes_motor.setPosition(self.z_axes_tgt)
        elif twist.linear.z < 0:
            self.z_axes_tgt = 0 if self.z_axes_tgt <= 0 else self.z_axes_tgt - 0.001
            self.z_axes_motor.setPosition(self.z_axes_tgt)

        if twist.linear.y > 0:
            self.y_axes_tgt += 0.001
            if self.y_axes_tgt > 0.2:
                self.y_axes_tgt = 0.2
            self.y_axes_motor.setPosition(self.y_axes_tgt)
        elif twist.linear.y < 0:
            self.y_axes_tgt = -0.1 if self.y_axes_tgt <= -0.1 else self.y_axes_tgt - 0.001
            self.y_axes_motor.setPosition(self.y_axes_tgt)

        if twist.linear.x > 0:
            self.x_axes_tgt += 0.001
            if self.x_axes_tgt > 0.2:
                self.x_axes_tgt = 0.2
            self.x_axes_motor.setPosition(self.x_axes_tgt)
        elif twist.linear.x < 0:
            self.x_axes_tgt = 0 if self.x_axes_tgt <= 0 else self.x_axes_tgt - 0.001
            self.x_axes_motor.setPosition(self.x_axes_tgt)

    def _base_apply_speeds_cb(self, twist):
        v_x = twist.linear.x
        v_y = twist.linear.y
        omega = twist.angular.z
        v_x /= self._wheel_radius
        v_y /= self._wheel_radius
        omega *= self._wheel_distance/ self._wheel_radius
        self._v_x = v_x
        self._omega = omega
        self.motor0.setVelocity(v_y - omega)
        self.motor1.setVelocity(-math.sqrt(0.75) * v_x - 0.5 * v_y - omega)
        self.motor2.setVelocity(math.sqrt(0.75) * v_x - 0.5 * v_y - omega)
    
    def step(self, ms):
        super().step(32)
        stamp = Time(seconds=self.robot.getTime()).to_msg()        
        time_diff_s = self.robot.getTime() - self._last_odometry_sample_time

        wheel0_ticks = self.motor0_posSensor.getValue()
        wheel1_ticks = self.motor1_posSensor.getValue()
        wheel2_ticks = self.motor2_posSensor.getValue()

        wheel0_rad_s = (wheel0_ticks - self._prev_wheel0_ticks) / time_diff_s
        wheel1_rad_s = (wheel1_ticks - self._prev_wheel1_ticks) / time_diff_s
        wheel2_rad_s = (wheel2_ticks - self._prev_wheel2_ticks) / time_diff_s

        wheel0_rpm = wheel0_rad_s * 9.5493
        wheel1_rpm = wheel1_rad_s * 9.5493
        wheel2_rpm = wheel2_rad_s * 9.5493

        m1 = wheel2_rpm
        m2 = wheel0_rpm
        m3 = wheel1_rpm

        #Convert from RPM to mm/s
        k = 60.0 * self._gear / (2* math.pi* self._wheel_radius)
        
        vx = -(m3 - m1) / math.sqrt(3.0) / k
        vy = -((2.0 / 3.0) * (m1 + 0.5 * (m3 - m1) - m2) / k)
        vw = -vy + m2 / k
        omega = -(vw / self._wheel_distance)
        
        self._prev_wheel0_ticks = wheel0_ticks
        self._prev_wheel1_ticks = wheel1_ticks
        self._prev_wheel2_ticks = wheel2_ticks
        self._last_odometry_sample_time = self.robot.getTime()

        res_phi = self._prev_odom_omega + omega * time_diff_s
        x = self._prev_odom_x + (vx * cos(res_phi) - vy * sin(res_phi)) * time_diff_s
        y = self._prev_odom_y + (vx * sin(res_phi) + vy * cos(res_phi)) * time_diff_s
        # TODO: add sin/cos of res_phi

        q = [0.0, 0.0, sin(res_phi/2), cos(res_phi/2)]

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = omega
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self._odometry_publisher.publish(msg)
        
        # Pack & publish transforms
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(tf)

        #q = [0.559507, -0.577502, -0.594511, 2.09486]
        #q = [-0.500, 0.500, 0.500, -0.500]
        q = [1.000, -0.000, 0.000, -0.000]
        translation = [0.005, 0.000, 0.373]
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'base_laser'
        #tf.transform.translation.x = -0.028  
        #tf.transform.translation.y = 0.03
        #tf.transform.translation.z = 0.16
        tf.transform.translation.x = translation[0]
        tf.transform.translation.y = translation[1]
        tf.transform.translation.z = translation[2]
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self._tf_broadcaster.sendTransform(tf)


        self._prev_odom_x = x
        self._prev_odom_y = y
        self._prev_odom_omega = res_phi

        #Gripper axis transfroms
        x_axis = self.robot.getFromDef("x_endpoint")
        x_trans = x_axis.getField("translation")
        x_trans_val = x_trans.getSFVec3f()

        x_rot = x_axis.getField("rotation")
        x_rot_val = x_rot.getSFRotation()
        #self.get_logger().info(str(x_rot_val))
        tf_x_axis = TransformStamped()
        tf_x_axis.header.stamp = stamp
        tf_x_axis.header.frame_id = 'x_axis_solid'
        tf_x_axis.child_frame_id = 'x_axis_endpoint'
        tf_x_axis.transform.translation.x = x_trans_val[0]
        tf_x_axis.transform.translation.y = x_trans_val[1]
        tf_x_axis.transform.translation.z = x_trans_val[2]
        tf_x_axis.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_x_axis)

        y_axis = self.robot.getFromDef("y_endpoint")
        y_trans = y_axis.getField("translation")
        y_trans_val = y_trans.getSFVec3f()
        tf_y_axis = TransformStamped()
        tf_y_axis.header.stamp = stamp
        tf_y_axis.header.frame_id = 'y_axis_solid'
        tf_y_axis.child_frame_id = 'y_axis_endpoint'
        tf_y_axis.transform.translation.x = y_trans_val[0]
        tf_y_axis.transform.translation.y = y_trans_val[1]
        tf_y_axis.transform.translation.z = y_trans_val[2]
        tf_y_axis.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_y_axis)

        z_axis = self.robot.getFromDef("z_endpoint")
        z_trans = z_axis.getField("translation")
        z_trans_val = z_trans.getSFVec3f()
        tf_z_axis = TransformStamped()
        tf_z_axis.header.stamp = stamp
        tf_z_axis.header.frame_id = 'z_axis_solid'
        tf_z_axis.child_frame_id = 'z_axis_endpoint'
        tf_z_axis.transform.translation.x = z_trans_val[0]
        tf_z_axis.transform.translation.y = z_trans_val[1]
        tf_z_axis.transform.translation.z = z_trans_val[2]
        tf_z_axis.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_z_axis)

        #belt_start_tf = self.robot.getFromDef("Belt_start_tf")
        #robo_tf = self.robot.getFromDef("Robotino3")
        #finger_tf = self.robot.getFromDef("finger_mounting")

        #trans_finger_tf = finger_tf.getField("translation")
        #trans_robo_tf = robo_tf.getField("translation")
        #trans_start_tf = belt_start_tf.getField("translation")

        #t_val_finger = trans_finger_tf.getSFVec3f()
        #values = trans_start_tf.getSFVec3f()
        #values_robot = trans_robo_tf.getSFVec3f()

        #Euklid Distance between Belt Input and Finger Mounting
        #finger_base_trans = [values_robot[0] + t_val_finger[0], values_robot[1] + t_val_finger[1],values_robot[2] + t_val_finger[2]  ]

        #distance = math.pow( (finger_base_trans[0] - values[0]) , 2) +math.pow( (finger_base_trans[1] - values[1]) ,2)+math.pow((finger_base_trans[2] - values[2]),2)
        #dist = Float32()
        #dist.data = distance
        #self._mps_distance_pub.publish(dist)

def main(args=None):
    rclpy.init(args=args)
    driver = RobotinoDriver(name = 'robotino3' ,args=args)
    rclpy.spin(driver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
