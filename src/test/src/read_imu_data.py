#!/usr/bin/env python3

import time
import serial 
import rospy

from std_msgs.msg import String

def main():
    rospy.init_node('imu_serial')
    pub = rospy.Publisher('/imu_serial', String, queue_size=10)
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
            msg = ",".join(real2)  # 리스트를 문자열로 변환
            pub.publish(String(msg))
            print(msg)  # 값 출력
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
