#!/usr/bin/env python3

import rospy
import serial as pyserial
from arduino_serial_communication.msg import Therster

ser = None  # 전역 변수로 선언하여 콜백 함수와 메인 함수에서 공유할 수 있도록 함

def therester_callback(data):
    global ser

    if ser is not None and ser.is_open:
        # 데이터를 쉼표로 구분하여 시리얼 포트로 전송
        data_str = ','.join([str(value) for value in data.Therster])
        data_str += 'b'

        data_str = "/" + data_str
        ser.write(data_str.encode('utf-8'))
        print("Data sent:", data_str)

        # 시리얼 데이터를 읽어서 콘솔에 출력
        received_data = ''
        while ser.in_waiting or received_data == '':
            received_data += ser.readline().decode()

        if received_data.startswith('&') and received_data.endswith('a'):
            print("Received data:", received_data.strip())
        else:
            print("Invalid data format:", received_data.strip())

def therester_serial_communication_node():
    global ser

    # 시리얼 포트 설정 (ttyUSB0 대신에 사용하는 포트명을 적절히 변경해주세요.)
    ser = pyserial.Serial('/dev/ttyACM0', 1000000)  # 포트 이름과 보드의 전송 속도에 맞게 수정

    rospy.init_node('therester_serial_communication_node', anonymous=True)
    rospy.Subscriber('/therster_topic', Therster, therester_callback)
    rospy.spin()

    # 노드 종료 시 시리얼 포트 닫기
    if ser is not None and ser.is_open:
        ser.close()

if __name__ == '__main__':
    try:
        therester_serial_communication_node()
    except rospy.ROSInterruptException:
        pass
