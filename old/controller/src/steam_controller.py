#!/usr/bin/env python

from __future__ import division
import math
from struct import pack
from threading import Thread
from steamcontroller import SteamController, SCButtons, SCStatus, SCI_NULL
import usb1
from time import sleep
import serial
from cobs import cobs


AXIS_MIN = -32768
AXIS_MAX = 32767

RIGHT_PAD = 0
LEFT_PAD = 1

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
    send(port, clamp(LF), clamp(-RF), clamp(LB), clamp(-RB))

def linear_map(value, inMin, inMax, outMin, outMax):
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def constrain(value, min_value, max_value):
    return min(max_value, max(min_value, value))

def remap_axis(x, y, left_pad=False, right_pad=False):
    x = linear_map(x, AXIS_MIN, AXIS_MAX, -1, 1)
    y = linear_map(y, AXIS_MIN, AXIS_MAX, -1, 1)
    if left_pad or right_pad:
        distance = math.hypot(x, y)
        corrected_angle = 0
        if left_pad:
            corrected_angle = math.atan2(x, y) + 0.1 * math.pi
        elif right_pad:
            corrected_angle = math.atan2(x, y) - 0.1 * math.pi
        x = math.sin(corrected_angle) * distance
        y = math.cos(corrected_angle) * distance
    x = constrain(x, -1, 1)
    y = constrain(y, -1, 1)
    return x, y

def scale_trigger(value):
    return linear_map(value, 0, 255, 0, 1)

def check_button(buttons, button_flag):
    return buttons & button_flag == button_flag


class HamiltonSteamController(SteamController):
    def run(self):
        try:
            while any(x.isSubmitted() for x in self._transfer_list):
                self._ctx.handleEvents()
                if len(self._cmsg) > 0:
                    cmsg = self._cmsg.pop()
                    self._sendControl(cmsg)
        except usb1.USBError as e:
            print("USB comm erro " + str(e))

class SteamControllerHandler(object):
    def __init__(self):
        super(SteamControllerHandler, self).__init__()
        self._previous_controller_data = SCI_NULL
        self.port = serial.Serial('/dev/ttyACM0', 115200)

        self._left_pad_moved = 0
        self._right_pad_moved = 0
        self.x = 0
        self.y = 0
        self.side = 0

        self.sc = HamiltonSteamController(self.on_controller_data)
        self.publisher_thread = Thread(target=self.publisher_loop)
        self.publisher_thread.start()
        self.sc.run()

    def on_controller_data(self, controller, controller_data):

        if controller_data.status != SCStatus.INPUT:
            return

        previous_data = self._previous_controller_data
        self._previous_controller_data = controller_data

        _xor = previous_data.buttons ^ controller_data.buttons
        buttons = controller_data.buttons
        buttons_lifted = _xor & previous_data.buttons
        buttons_pressed = _xor & controller_data.buttons

        left_grip_down = bool(buttons & SCButtons.LGRIP)
        right_grip_down = bool(buttons & SCButtons.RGRIP)

        # pads
        robot_x = 0
        robot_y = 0
        robot_rot = 0

        if check_button(buttons, SCButtons.LPADTOUCH):
            x, y = remap_axis(controller_data.lpad_x, controller_data.lpad_y, left_pad=True)
            xp, yp = remap_axis(previous_data.lpad_x, previous_data.lpad_y, left_pad=True)
            self._left_pad_moved += math.sqrt((x - xp)**2 + (y - yp)**2)
            if self._left_pad_moved >= 0.1:
                controller.addFeedback(LEFT_PAD, amplitude=150)
                self._left_pad_moved %= 0.1
            robot_x = -x
            robot_y = y
            # print "Left pad is at X:{0:.3f} Y:{1:.3f}".format(x, y)

        if check_button(buttons, SCButtons.RPADTOUCH):
            x, y = remap_axis(controller_data.rpad_x, controller_data.rpad_y, right_pad=True)
            xp, yp = remap_axis(previous_data.rpad_x, previous_data.rpad_y, right_pad=True)
            self._right_pad_moved += math.sqrt((x - xp)**2 + (y - yp)**2)
            if self._right_pad_moved >= 0.1:
                controller.addFeedback(RIGHT_PAD, amplitude=150)
                self._right_pad_moved %= 0.1
            robot_rot = x
            # print "Right pad is at X:{0:.3f} Y:{1:.3f}".format(x, y)

        if not check_button(buttons, SCButtons.LPADTOUCH) and (controller_data.lpad_x != 0 or controller_data.lpad_y != 0):
            x, y = remap_axis(controller_data.lpad_x, controller_data.lpad_y)
            # print "Stick is at X:{0:.3f} Y:{1:.3f}".format(x, y)
        
        self.x = robot_x
        self.y = robot_y
        self.side = robot_rot


    def publisher_loop(self):
        while True:
            dir(self.port, deadzone(self.y), deadzone(self.x), deadzone(-self.side))
            sleep(0.01)

if __name__ == "__main__":
    SteamControllerHandler()
