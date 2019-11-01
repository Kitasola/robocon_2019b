#include <Adafruit_NeoPixel.h>
#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#include <Servo.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN         10
#define NUMPIXELS   16
#define READ_PIN_1  A3
#define READ_PIN_2  A2
#define SERVO_PIN_1 9
#define SERVO_PIN_2 3
#define HEAD_SERVO_1 2
#define HEAD_SERVO_2  4

int head_flag = 0;


void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//const number

constexpr int Motor[3][3] = {
  {5, 6, 12},
  {10, 11, 13},
  {9, 3, 2}
};
constexpr long int BAUDRATE = 115200;
constexpr int REDE_PIN = 4;
constexpr int high_angle_1 = 150, low_angle_1 = 30, high_angle_2 = 65, low_angle_2 = 30;
//yellow = (255,255,0), green = (0,255,0), blue = (0,0,255),red = (255,0,0), Non = (0,0,0) White = (255,255,255)
int f2 = analogRead(READ_PIN_2);

constexpr int color[5][3] = {
  {  0,  0,  0},//Non
  {255, 255,  0}, //yellow
  {  0, 255,  0}, //green
  {255,  0,  0},//red
  {  0,  0, 255} //blue
};

ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);
Servo servo_1;
Servo servo_2;
Servo head_servo_1;
Servo head_servo_2;

int angle;
int stop_flag = 0;
int prev_flag = 0;

void setup() {
  pinMode(REDE_PIN, OUTPUT);
  pinMode(READ_PIN_1, INPUT);
  pinMode(READ_PIN_2, INPUT);
  head_servo_1.attach(HEAD_SERVO_1);
  head_servo_2.attach(HEAD_SERVO_2);
  servo_1.attach(SERVO_PIN_1);
  servo_2.attach(SERVO_PIN_2);
  Serial.begin(BAUDRATE);
  slave.addCMD(100, set);
  slave.addCMD(40, servo);
  slave.addCMD(250, headMove);
  pixels.begin();
  pixels.setBrightness(100);
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  if (analogRead(READ_PIN_1) > 512) {
    prev_flag = 1;
  } else {
    prev_flag = 0;
  }

  head_servo_1.write(55);
  head_servo_2.write(125);

}

int led_flag = 0;

boolean set(int rx_data, int& tx_data) {
  led_flag = rx_data;
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(color[led_flag][0], color[led_flag][1], color[led_flag][2]));
  }
  pixels.show();
  return true;
}

boolean servo(int rx_data, int& tx_data) {
  angle = rx_data;
  stop_flag = 1;
  servo_1.write(angle);
  return true;
}

boolean headMove(int rx_data, int& tx_data) {
  if (head_flag == 0) {
    if (rx_data == 1) {
      head_servo_1.write(90);
      head_servo_2.write(90);
      unsigned long start_time = millis();
      while (millis() < start_time + 1000) {

      }
      head_servo_1.write(150);
      head_servo_2.write(30);
    }
  }
  head_flag = 1;
  return true;
}


void loop() {
  slave.check();
  int cur_flag = 0;
  int f1 = analogRead(READ_PIN_1);
  if (f1 > 512) {
    cur_flag = 1;
  } else {
    cur_flag = 0;
  }

  int f2 = analogRead(READ_PIN_2);

  if (prev_flag != cur_flag) {
    stop_flag = 0;
  }

  if (stop_flag == 0) {
    if (f1 > 512) {
      servo_1.write(high_angle_1);
    } else if (f1 < 10) {
      servo_1.write(low_angle_1);
    }
  }

  if (f2 > 512) {
    servo_2.write(high_angle_2);
  } else if (f2 < 10) {
    servo_2.write(low_angle_2);
  }
  prev_flag = cur_flag;
}
