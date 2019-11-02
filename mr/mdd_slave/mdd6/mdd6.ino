#include <Adafruit_NeoPixel.h>
#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#include <Servo.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

constexpr long int BAUDRATE = 115200;
constexpr int REDE_PIN = 4;

#define PIN         10
#define NUMPIXELS   16
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
constexpr int color[5][3] = {
  {  0,  0,  0},//Non
  {255, 255,  0}, //yellow
  {  0, 255,  0}, //green
  {255,  0,  0},//red
  {  0,  0, 255} //blue
};

int head_flag = 0;
constexpr int SHOOT_READ_PIN[2] = {A3, A2};
int current_shoot_read[2] = {}, prev_shoot_read[2] = {};
constexpr int SHOOT_READ_THRESHOULD = 512;

constexpr int SHOOT_SERVO_PIN[2] = {9, 3};
constexpr int SHOOT_SERVO_FREE[2] = {30, 90}, SHOOT_SERVO_ROCK[2] = {150, 135};
Servo shoot_servo[2];

constexpr int HEAD_SERVO_PIN[2] = {5, 6};
constexpr int HEAD_SERVO_OPEN[2] = {55, 125}, HEAD_SERVO_WAIT[2] = {90, 90}, HEAD_SERVO_CLOSE[2] = {125, 55};
constexpr int WAIT_HEAD_TIME = 1000;
Servo head_servo[2];
int head_phase = 0;
unsigned long head_start_time;

ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);

int angle;
int stop_flag = 0;

void setup() {
  pinMode(REDE_PIN, OUTPUT);
  for (int i = 0; i < 2; ++i) {
    pinMode(SHOOT_READ_PIN[i], INPUT);
    if (analogRead(SHOOT_READ_PIN[i]) > SHOOT_READ_THRESHOULD) {
      current_shoot_read[i] = 1;
    } else {
      current_shoot_read[i] = 0;
    }
    shoot_servo[i].attach(SHOOT_SERVO_PIN[i]);
    shoot_servo[i].write(SHOOT_SERVO_FREE[i]);
    head_servo[i].attach(HEAD_SERVO_PIN[i]);
    head_servo[i].write(HEAD_SERVO_OPEN[i]);
  }
  Serial.begin(BAUDRATE);
  slave.addCMD(40, actServo);
  slave.addCMD(100, setColor);
  slave.addCMD(250, actHead);
  pixels.begin();
  pixels.setBrightness(100);
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

}

void loop() {
  slave.check();
  for(int i = 0; i < 2; ++i) {
    prev_shoot_read[i] = current_shoot_read[i];
    if(analogRead(SHOOT_READ_PIN[i]) > SHOOT_READ_THRESHOULD){
      current_shoot_read[i] = 1;
    } else {
      current_shoot_read[i] = 0;  
    }
    if(current_shoot_read[i] == 1 && prev_shoot_read[i] == 0) {
      shoot_servo[i].write(SHOOT_SERVO_FREE[i]);  
    } else if(current_shoot_read[i] == 0 && prev_shoot_read[i] == 1) {
      shoot_servo[i].write(SHOOT_SERVO_ROCK[i]);  
    }
  }

  switch (head_phase) {
    case 0:
      break;
    case 1:
      for (int i = 0; i < 2; ++i) {
        head_servo[i].write(HEAD_SERVO_WAIT[i]);
      }
      head_start_time = millis();
      head_phase = 2;
      break;
    case 2:
      if (millis() > head_start_time + WAIT_HEAD_TIME) {
        head_phase = 3;
      }
      break;
    case 3:
      for (int i = 0; i < 2; ++i) {
        head_servo[i].write(HEAD_SERVO_CLOSE[i]);
      }
      head_phase = 4;
      break;
    case 4:
      break;
  }
}

boolean setColor(int rx_data, int& tx_data) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(color[rx_data][0], color[rx_data][1], color[rx_data][2]));
  }
  pixels.show();
  return true;
}

boolean actServo(int rx_data, int& tx_data) {
  shoot_servo[0].write(rx_data);
  return true;
}

boolean actHead(int rx_data, int& tx_data) {
  head_phase = rx_data;
  return true;
}
