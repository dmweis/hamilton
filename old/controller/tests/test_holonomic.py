from __future__ import division

from cobs import cobs
from struct import pack

def clamp(value, a=-255, b=255):
    min_value = min(a, b)
    max_value = max(a, b)
    return int(max(min(max_value, value), min_value))

def send(port, LF, RF, LB, RB):
    command = [int(LF>0), abs(LF), int(RF>0), abs(RF), int(LB>0), abs(LB), int(RB>0), abs(RB)]
    data = cobs.encode(bytes(bytearray(command))) + bytes(bytearray([0]))
    return data


def dir(port, forward, strafe, rot):
    RF = (forward + rot + strafe)*255
    RB = (forward + rot - strafe)*255
    LF = (forward - rot - strafe)*255
    LB = (forward - rot + strafe)*255
    return send(port, clamp(LF), clamp(-RF), clamp(LB), clamp(-RB))


def test_forward():
    command = send(None, 255, 0, 0, 0)
    assert command == b'\x03\x01\xff\x01\x01\x01\x01\x01\x01\x00'


def test_backwards():
    command = send(None, -255, 0, 0, 0)
    assert command == b'\x01\x02\xff\x01\x01\x01\x01\x01\x01\x00'
