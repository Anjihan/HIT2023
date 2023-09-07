#!/usr/bin/env python3

import time
import serial
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Imu

def main():
    rospy.init_node('imu_plot_juggler')
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.001)

    while not rospy.is_shutdown():
        testString = b""  # 변수 초기화
        if ser.inWaiting():
            time.sleep(0.05)
            while ser.inWaiting():
                data = ser.read()
                testString += data
            real = testString[1:-2].decode('utf-8')
            real2 = real.split(",")

            if len(real2) == 9:  # 예상되는 값의 개수에 따라 수정 필요
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = rospy.Time.now()
                
                # 필요한 데이터만 할당
                imu_msg.orientation.x = float(real2[0])  # Roll
                imu_msg.orientation.y = float(real2[1])  # Pitch
                imu_msg.orientation.z = float(real2[2])  # Yaw
                
                imu_msg.angular_velocity.x = float(real2[3])  # 각속도(x)
                imu_msg.angular_velocity.y = float(real2[4])  # 각속도(y)
                imu_msg.angular_velocity.z = float(real2[5])  # 각속도(z)
                
                imu_msg.linear_acceleration.x = float(real2[6])  # 선속도(x)
                imu_msg.linear_acceleration.y = float(real2[7])  # 선속도(y)
                imu_msg.linear_acceleration.z = float(real2[8])  # 선속도(z)
                
                pub.publish(imu_msg)
                print(imu_msg.orientation)
                print(imu_msg.angular_velocity)
                print(imu_msg.linear_acceleration)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
