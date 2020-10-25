#include <PacketSerial.h>
#include <AccelStepper.h>

PacketSerial_<COBS, 0, 500> packetSerial;

AccelStepper RightFrontWheel(1, 2, 5);
AccelStepper LeftFrontWheel(1, 4, 7);
AccelStepper RightBackWheel(1, 3, 6);
AccelStepper LeftBackWheel(1, 12, 13);

void motorsOff() {
  digitalWrite(8, HIGH);
}

void motorsOn() {
  digitalWrite(8, LOW);
}

void setup()
{
  pinMode(8, OUTPUT);
  motorsOff();
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);

  LeftFrontWheel.setMaxSpeed(300);
  LeftBackWheel.setMaxSpeed(300);
  RightFrontWheel.setMaxSpeed(300);
  RightBackWheel.setMaxSpeed(300);

  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);
}
void loop()
{
  packetSerial.update();
  // Execute the steps
  LeftFrontWheel.runSpeed();
  LeftBackWheel.runSpeed();
  RightFrontWheel.runSpeed();
  RightBackWheel.runSpeed();
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

    if (tempBuffer[1] == 0 && tempBuffer[3] == 0 && tempBuffer[5] == 0 && tempBuffer[7] == 0){
      motorsOff();
    } else {
      motorsOn();
    }

    if (tempBuffer[0])
    {
        LeftFrontWheel.setSpeed(tempBuffer[1]);
    }
    else
    {
        LeftFrontWheel.setSpeed(-tempBuffer[1]);
    }

    if (tempBuffer[2])
    {
        RightFrontWheel.setSpeed(tempBuffer[3]);
    }
    else
    {
        RightFrontWheel.setSpeed(-tempBuffer[3]);
    }

    if (tempBuffer[4])
    {
        LeftBackWheel.setSpeed(tempBuffer[5]);
    }
    else
    {
        LeftBackWheel.setSpeed(-tempBuffer[5]);
    }

    if (tempBuffer[6])
    {
        RightBackWheel.setSpeed(tempBuffer[7]);
    }
    else
    {
        RightBackWheel.setSpeed(-tempBuffer[7]);
    }
}
