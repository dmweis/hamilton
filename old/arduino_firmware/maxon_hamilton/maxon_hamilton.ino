#include <PacketSerial.h>

PacketSerial_<COBS, 0, 500> packetSerial;

void setup()
{
    packetSerial.begin(115200);
    packetSerial.setPacketHandler(&onPacketReceived);

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);

    setMotorLB(0);
    setMotorRB(0);
    setMotorLF(0);
    setMotorRF(0);
}

void loop()
{
    packetSerial.update();
}

void onPacketReceived(const uint8_t *buffer, size_t size)
{
    if (size != 8)
    {
        return;
    }
    // Make a temporary buffer.
    uint8_t tempBuffer[size];

    // Copy the packet into our temporary buffer.
    memcpy(tempBuffer, buffer, size);

    if (tempBuffer[0])
    {
        setMotorLF(tempBuffer[1]);
    }
    else
    {
        setMotorLF(-tempBuffer[1]);
    }

    if (tempBuffer[2])
    {
        setMotorRF(tempBuffer[3]);
    }
    else
    {
        setMotorRF(-tempBuffer[3]);
    }

    if (tempBuffer[4])
    {
        setMotorLB(tempBuffer[5]);
    }
    else
    {
        setMotorLB(-tempBuffer[5]);
    }

    if (tempBuffer[6])
    {
        setMotorRB(tempBuffer[7]);
    }
    else
    {
        setMotorRB(-tempBuffer[7]);
    }
}

void setMotor(int pinA, int pinB, int pinPwm, int value)
{
    if (value == 0)
    {
        digitalWrite(pinA, LOW);
        analogWrite(pinPwm, 0);
        digitalWrite(pinB, LOW);
    }
    else if (value > 0)
    {
        digitalWrite(pinA, HIGH);
        analogWrite(pinPwm, value);
        digitalWrite(pinB, LOW);
    }
    else if (value < 0)
    {
        digitalWrite(pinA, LOW);
        analogWrite(pinPwm, -value);
        digitalWrite(pinB, HIGH);
    }
}

void setMotorRB(int value)
{
    setMotor(2, 4, 3, value);
}

void setMotorLB(int value)
{
    setMotor(5, 7, 6, value);
}

void setMotorLF(int value)
{
    setMotor(8, 13, 9, value);
}

void setMotorRF(int value)
{
    setMotor(10, 12, 11, value);
}
