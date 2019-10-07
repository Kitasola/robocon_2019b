#include <Adafruit_NeoPixel.h>
#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define PIN        6
#define NUMPIXELS 16

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

Adafruit_NeoPixel pixels[2] = {Adafruit_NeoPixel(NUMPIXELS, 5, NEO_GRB + NEO_KHZ800), Adafruit_NeoPixel(NUMPIXELS, 6, NEO_GRB + NEO_KHZ800)};

//const number

constexpr int Motor[3][3] = {
  {5, 6, 12},
  {10, 11, 13},
  {9, 3, 2}
};
constexpr long int BAUDRATE = 115200;
constexpr int REDE_PIN = 4;
//yellow = (255,255,0), green = (0,255,0), blue = (0,0,255),red = (255,0,0), Non = (0,0,0) White = (255,255,255)

constexpr int color[5][3] = {
  {  0,  0,  0},//Non
  {255, 255,  0}, //yellow
  {  0, 255,  0}, //green
  {255,  0,  0},//red
  {  0,  0, 255} //blue
};

ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);

void setup() {
  pinMode(REDE_PIN, OUTPUT);
  Serial.begin(BAUDRATE);
  slave.addCMD(100, set0);
  slave.addCMD(101, set1);
  for (int i = 0; i < 2; ++i) {
    pixels[i].begin();
    pixels[i].setBrightness(100);
  }
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
}

int led_flag = 0;

boolean set0(int rx_data, int& tx_data) {
  led_flag = rx_data;
  for (int i = 0; i <  NUMPIXELS; i++) {
    pixels[0].setPixelColor(i, pixels[0].Color(color[led_flag][0], color[led_flag][1], color[led_flag][2]));
  }
  pixels[0].show();
  return true;
}

boolean set1(int rx_data, int& tx_data) {
  led_flag = rx_data;
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels[1].setPixelColor(i, pixels[1].Color(color[led_flag][0], color[led_flag][1], color[led_flag][2]));
  }
  pixels[1].show();
  return true;
}

void loop() {
  slave.check();

}
