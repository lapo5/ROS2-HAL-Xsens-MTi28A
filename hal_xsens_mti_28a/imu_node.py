import time

import rclpy
from rclpy.node import Node
import numpy as np
import sys
import math
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField

import os

from resources import xsens_MTi28A

# Class definition fo the estimator
class IMU_Node(Node):
    def __init__(self):
        super().__init__("xsens_imu_node")

        self.imu = xsens_MTi28A.HAL()

        if not self.imu.init(xsens_MTi28A.Mode.CalibratedData, "/dev/ttyUSB0"):
            sys.exit(1)
        

        self.rotvel = np.array([0, 0, 0], dtype=np.float32)
        self.linacc = np.array([0, 0, 0], dtype=np.float32)
        self.magfld = np.array([0, 0, 0], dtype=np.float32)
        
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/magnetic", 10)

        self.timer = self.create_timer(0.03, self.publish_status)



    # Publisher function
    def publish_status(self):

        self.imu.get_gyro_acc_mag(self.rotvel, self.linacc, self.magfld)
        
        msg = Imu()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "imu_link"

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0

        msg.orientation_covariance[0] = 0.01745
        msg.orientation_covariance[4] = 0.01745
        msg.orientation_covariance[8] = 0.15708

        msg.angular_velocity.x = float(self.rotvel[0])
        msg.angular_velocity.y = float(self.rotvel[1])
        msg.angular_velocity.z = float(self.rotvel[2])

        msg.angular_velocity_covariance[0] = 0.0004
        msg.angular_velocity_covariance[4] = 0.0004
        msg.angular_velocity_covariance[8] = 0.0004

        msg.linear_acceleration.x = float(self.linacc[0])
        msg.linear_acceleration.y = float(self.linacc[1])
        msg.linear_acceleration.z = float(self.linacc[2])

        msg.linear_acceleration_covariance[0] = 0.0004
        msg.linear_acceleration_covariance[4] = 0.0004
        msg.linear_acceleration_covariance[8] = 0.0004

        self.imu_pub.publish(msg)

        '''
        mag_msg = MagneticField()

        mag_msg.header = Header()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "imu_mag_link"

        mag_msg.magnetic_field.x = float(self.magfld[0])
        mag_msg.magnetic_field.y = float(self.magfld[1])
        mag_msg.magnetic_field.z = float(self.magfld[2])

        mag_msg.magnetic_field_covariance[0] = 0.1
        mag_msg.magnetic_field_covariance[4] = 0.1
        mag_msg.magnetic_field_covariance[8] = 0.1

        self.mag_pub.publish(mag_msg)
        '''


# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = IMU_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Xsens IMU Node stopped cleanly')
    except BaseException:
        print('Exception in Xsens IMU Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.destroy_node()
        rclpy.shutdown()

# Main
if __name__ == '__main__':
    main()
