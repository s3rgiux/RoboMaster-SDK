#!python3

import sys
sys.path.append('../decoder/ubuntu/output/')
sys.path.append('../../connection/network/')

import threading
import time
import numpy as np
import libh264decoder
import signal
from PIL import Image as PImage
import cv2
import opus_decoder
import pyaudio
import robot_connection
import enum
import queue


class ConnectionType(enum.Enum):
    WIFI_DIRECT = 1
    WIFI_NETWORKING = 2
    USB_DIRECT = 3


class RobotLiveview(object):
    WIFI_DIRECT_IP = '192.168.2.1'
    WIFI_NETWORKING_IP = '192.168.0.164'
    USB_DIRECT_IP = '192.168.42.2'
        
    def __init__(self, connection_type):
        self.connection = robot_connection.RobotConnection()
        self.connection_type = connection_type
        self.is_shutdown = True

    def open(self):
        if self.connection_type is ConnectionType.WIFI_DIRECT:
            self.connection.update_robot_ip(RobotLiveview.WIFI_DIRECT_IP)
        elif self.connection_type is ConnectionType.USB_DIRECT:
            self.connection.update_robot_ip(RobotLiveview.USB_DIRECT_IP)
        elif self.connection_type is ConnectionType.WIFI_NETWORKING:
            robot_ip = self.connection.get_robot_ip(timeout=10)  
            print("got robot ip ")
            print(robot_ip)
            if robot_ip:
                self.connection.update_robot_ip(robot_ip)
            else:
                print('Get robot failed')
                return False
        self.is_shutdown = not self.connection.open()
        self.connection.test()
        
    def close(self):
        self.connection.close()
        self.is_shutdown = True

    def display(self):
        self.command('command')
        print("Entered in SDK")
        time.sleep(1)
        self.command('audio on')
        time.sleep(1)
        self.command('stream on')
        time.sleep(1)
        self.command('stream on')
        # self.video_decoder_thread.start()
        # self.video_display_thread.start()
        # self.audio_decoder_thread.start()
        # self.audio_display_thread.start()

        print('display!')

    def command(self, msg):
        # TODO: TO MAKE SendSync()
        #       CHECK THE ACK AND SEQ
        self.connection.send_data(msg)


def test():

    robot = RobotLiveview(ConnectionType.WIFI_NETWORKING) #ConnectionType.USB_DIRECT)

    def exit(signum, frame):
        robot.close()

    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)

    robot.open()
    #robot.display()
    # time.sleep(25)
    # robot.command('quit')
    robot.close()
    # 


def test2():
    """
    Test funciton

    Connect robot and query the version 
    """
    print("TEst_mine")
    robot = robot_connection.RobotConnection('192.168.0.164')
    robot.open()

    robot.send_data('command')
    print('send data to robot   : command')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)

    robot.send_data('version ?')
    print('send data to robot   : version ?')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)

    robot.send_data('stream on')
    print('send data to robot   : stream on')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)
    result = robot.start_video_recv()
    if result:
        stream_data = robot.recv_video_data(5)
        print('recv video data from robot %s'%stream_data)
        robot.stop_video_recv()
    robot.send_data('stream off')
    print('send data to robot   : stream off')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)

    robot.send_data('audio on')
    print('send data to robot   : audio on')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)
    result = robot.start_audio_recv()
    if result:
        stream_data = robot.recv_audio_data(5)
        print('recv audio data from robot %s'%stream_data)
        robot.stop_audio_recv()
    robot.send_data('audio off')
    print('send data to robot   : audio off')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)

    robot.send_data('quit')
    print('send data to robot   : quit')
    recv = robot.recv_ctrl_data(5)
    print('recv data from robot : %s'%recv)

    robot.close()


if __name__ == '__main__':
    # test()
    test2()