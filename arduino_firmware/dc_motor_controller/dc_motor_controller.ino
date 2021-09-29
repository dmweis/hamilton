#include <PacketSerial.h>
#include "Motor.h"

volatile long lastUpdateTime = 0;

long commandTimeout = 200;

PacketSerial_<COBS, 0, 500> packetSerial;

Motor motorA = Motor(2, 4, 3);
Motor motorB = Motor(7, 8, 5);
Motor motorC = Motor(17, 18, 6);
Motor motorD = Motor(19, 14, 9);

void setup()
{
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);
}

void loop()
{
  packetSerial.update();
  if (millis() - lastUpdateTime > commandTimeout)
  {
    motorA.set_speed(0);
    motorB.set_speed(0);
    motorC.set_speed(0);
    motorD.set_speed(0);
  }
}

void onPacketReceived(const uint8_t *buffer, size_t size)
{
  if (size != 8)
  {
    return;
  }
  lastUpdateTime = millis();
  // Make a temporary buffer.
  uint8_t tempBuffer[size];

  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);

  if (tempBuffer[0])
  {
    motorA.set_speed(tempBuffer[1]);
  }
  else
  {
    motorA.set_speed(-tempBuffer[1]);
  }

  if (tempBuffer[2])
  {
    motorB.set_speed(tempBuffer[3]);
  }
  else
  {
    motorB.set_speed(-tempBuffer[3]);
  }

  if (tempBuffer[4])
  {
    motorC.set_speed(tempBuffer[5]);
  }
  else
  {
    motorC.set_speed(-tempBuffer[5]);
  }

  if (tempBuffer[6])
  {
    motorD.set_speed(tempBuffer[7]);
  }
  else
  {
    motorD.set_speed(-tempBuffer[7]);
  }
}
