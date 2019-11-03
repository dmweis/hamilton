#!/usr/bin/env python

from __future__ import division

import serial

from cobs import cobs
from struct import pack
from time import sleep
from inputs import get_gamepad

def clamp(value, a=-255, b=255):
    min_value = min(a, b)
    max_value = max(a, b)
    return int(max(min(max_value, value), min_value))

def deadzone(value, deadzone=0.2):
    if value < deadzone and value > -deadzone:
        return 0.0
    return value

def send(port, LF, RF, LB, RB):
    command = [int(LF>0), abs(LF), int(RF>0), abs(RF), int(LB>0), abs(LB), int(RB>0), abs(RB)]
    data = cobs.encode(bytes(bytearray(command))) + bytes(bytearray([0]))
    port.write(data)

def dir(port, forward, strafe, rot):
    RF = (forward + rot + strafe)*255
    RB = (forward + rot - strafe)*255
    LF = (forward - rot - strafe)*255
    LB = (forward - rot + strafe)*255
    send(port, clamp(LF), clamp(RF), clamp(LB), clamp(RB))

if __name__ == "__main__":
    with serial.Serial('/dev/ttyACM0', 115200) as port:
        sleep(2)
        y = 0
        x = 0
        rot = 0
        try:
            while True:
                events = get_gamepad()
                debug = ""
                for event in events:
                    if event.code == "ABS_Y":
                        x = -(event.state / 32767.0)
                    if event.code == "ABS_X":
                        y = -(event.state / 32767.0)
                    if event.code == "ABS_RX":
                        rot = -(event.state / 32767.0)
                dir(port, deadzone(x), deadzone(y), deadzone(rot))
        except:
            dir(port, 0, 0, 0)
