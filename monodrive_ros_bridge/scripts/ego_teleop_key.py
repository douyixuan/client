#!/usr/bin/env python

import math
import rospy
import select
import sys
import termios

from ackermann_msgs.msg import AckermannDrive


import termios, tty


def clamp(min_value, n, max_value):
    return max(min_value, min(n, max_value))

def get_key(attrs):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if ord(key) == 27:
            key += sys.stdin.read(2)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attrs)

    return key

def send_message(publisher, steering, throttle):
    # convert to ackermann message
    max_angle = math.radians(40.0)
    max_speed = 27  # m/s
    angle = clamp(-max_angle,
                  max_angle * steering,  # ackermann left is positive
                  max_angle)
    angle_velocity = 0.0
    speed = clamp(-max_speed,
                  max_speed * throttle,
                  max_speed)
    accel = 0.0
    jerk = 0.0
    message = AckermannDrive(steering_angle=angle, steering_angle_velocity=angle_velocity,
                             speed=speed, acceleration=accel, jerk=jerk)
    publisher.publish(message)


if __name__=="__main__":
    attrs = termios.tcgetattr(sys.stdin)

    rospy.init_node('ego_teleop_key')
    publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

    throttle = 0.0
    steering = 0.0

    while(1):
        key = get_key(attrs)
        if key in ['\x03','\x04','\x1a']:
            break
        elif len(key)==3:
            if ord(key[2]) == 68:   # left
                steering -= 0.02

            elif ord(key[2]) == 67: # right
                steering += 0.02

            elif ord(key[2]) == 65: # up
                throttle += 0.05

            elif ord(key[2]) == 66: # down
                throttle -= 0.05

            print("throttle: {0}, steering: {1}".format(throttle, steering))
            send_message(publisher, steering, throttle)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attrs)
