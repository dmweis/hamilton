unsigned long last_serial = 0;

const byte interruptPinA = 2;

volatile int steps_since_last = 0;
volatile int current_speed = 0;
volatile int previous_speed = 0;
volatile int target_speed = 0;
volatile float speed_error = 0;
volatile float error_sum = 0;
volatile int set_speed = 0;

float Kp = 2.0;
float Kd = 0.0;
float Ki = 30.0;

#define PID_LOOP_TIME 0.01

void setup()
{
    pinMode(interruptPinA, INPUT_PULLUP);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    last_serial = millis();
    Serial.begin(115200);
    Serial.println("current_speed, target_speed, speed_error, error_sum");

    attachInterrupt(digitalPinToInterrupt(interruptPinA), countStep, RISING);
    start_timer();
}

void loop()
{
    unsigned long now = millis();
    if (now - last_serial > 200)
    {
        last_serial = millis();
        Serial.print(current_speed);
        Serial.print(", ");
        Serial.print(target_speed);
        Serial.print(", ");
        Serial.print(speed_error);
        Serial.print(", ");
        Serial.print(error_sum / 10.0);
        Serial.println();
    }
    if (Serial.available() > 0)
    {
        target_speed = Serial.parseInt();
        while (Serial.available())
        {
            Serial.read();
        }
    }
}

ISR(TIMER1_COMPA_vect)
{
    previous_speed = current_speed;
    current_speed = steps_since_last * 100 * 60 / 300;
    steps_since_last = 0;
    speed_error = target_speed - current_speed;

    error_sum += speed_error;
    error_sum = constrain(error_sum, -400, 1500);

    set_speed = Kp * speed_error + Ki * (error_sum)*PID_LOOP_TIME + Kd * (current_speed - previous_speed) / PID_LOOP_TIME;

    //set_speed = constrain(speed_error * 2, 0, 255);

    analogWrite(3, constrain(set_speed, 0, 255));
}

void countStep()
{
    steps_since_last++;
}

void start_timer()
{
    // these timers are set up with an Arduino Uno like board in mind
    // initialize Timer1
    cli();      // disable global interrupts
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    // set compare match register to set sample time 5ms
    OCR1A = 20000;
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for prescaling by 8
    TCCR1B |= (1 << CS11);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); // enable global interrupts
}