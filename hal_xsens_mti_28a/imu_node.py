import time

import rclpy
from rclpy.node import Node
import numpy as np
import sys
import math
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

import os

sys.path.append("/home/roxy/ros2_ws/src/ROS2-HAL-Xsens-MTi-28A/hal_xsens_mti_28a")
import xsens_MTi28A

# Class definition fo the estimator
class IMU_Node(Node):
    def __init__(self):
        super().__init__("ptu_controller")

        self.imu = xsens_MTi28A.HAL()
        if not self.imu.init(xsens_MTi28A.Mode.CalibratedData, "/dev/ttyUSB0"):
            sys.exit(1)
        

        self.rotvel = np.array([0, 0, 0], dtype=np.float32)
        self.linacc = np.array([0, 0, 0], dtype=np.float32)
        self.magfld = np.array([0, 0, 0], dtype=np.float32)
        
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/status", 10)
        self.timer = self.create_timer(0.03, self.publish_status)



    # Publisher function
    def publish_status(self):

        self.imu.get_gyro_acc_mag(self.rotvel, self.linacc, self.magfld)
        
        msg = Imu()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "IMU"

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0

        msg.angular_velocity.x = float(self.rotvel[0])
        msg.angular_velocity.y = float(self.rotvel[1])
        msg.angular_velocity.z = float(self.rotvel[2])

        msg.linear_acceleration.x = float(self.linacc[0])
        msg.linear_acceleration.y = float(self.linacc[1])
        msg.linear_acceleration.z = float(self.linacc[2])


        self.imu_pub.publish(msg)


# Main loop function
def main(args=None):
    rclpy.init(args=args)
    node = IMU_Node()
    rclpy.spin(node)
    rclpy.shutdown()


# Main
if __name__ == '__main__':
    main()
