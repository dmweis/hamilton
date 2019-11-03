
void setup() {
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

  /*digitalWrite(2, LOW);
  analogWrite(3, 100);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  analogWrite(6, 100);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  analogWrite(9, 100);
  digitalWrite(10, LOW);
  analogWrite(11, 100);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);*/
  setMotorLB(0);
  setMotorRB(0);
  setMotorLF(0);
  setMotorRF(0);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  if (Serial.available() > 0){
      char input = Serial.read();
      if (input == 'a'){
        setMotorLF(-150);
        setMotorRF(150);
        setMotorLB(150);
        setMotorRB(-150);
      }
      else if (input == 'd'){
        setMotorLF(150);
        setMotorRF(-150);
        setMotorLB(-150);
        setMotorRB(150);
      }
      else if (input == 'w'){
        setMotorLF(60);
        setMotorRF(60);
        setMotorLB(60);
        setMotorRB(60);
      }
      else if (input == 's'){
        setMotorLF(-60);
        setMotorRF(-60);
        setMotorLB(-60);
        setMotorRB(-60);
      }
      else if (input == 'e'){
        setMotorLF(70);
        setMotorRF(-70);
        setMotorLB(70);
        setMotorRB(-70);
      }
      else if (input == 'q'){
        setMotorLF(-70);
        setMotorRF(70);
        setMotorLB(-70);
        setMotorRB(70);
      }
      else{
        setMotorLF(0);
        setMotorRF(0);
        setMotorLB(0);
        setMotorRB(0);
      }
  }
}

void setMotor(int pinA, int pinB, int pinPwm, int value){
  if (value == 0){
    digitalWrite(pinA, LOW);
    analogWrite(pinPwm, 0);
    digitalWrite(pinB, LOW);
  }
  else if (value > 0){
    digitalWrite(pinA, HIGH);
    analogWrite(pinPwm, value);
    digitalWrite(pinB, LOW);
  }
  else if (value < 0){
    digitalWrite(pinA, LOW);
    analogWrite(pinPwm, -value);
    digitalWrite(pinB, HIGH);
  }
}

void setMotorRB(int value){
  setMotor(2, 4, 3, value);
}

void setMotorLB(int value){
  setMotor(5, 7, 6, value);
}

void setMotorLF(int value){
  setMotor(10, 8, 9, value);
}

void setMotorRF(int value){
  setMotor(12, 13, 11, value);
}

