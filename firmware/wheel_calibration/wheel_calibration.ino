// CleanMate - wheel encoder calibration (type 0, turn wheels 10 times, type c)

#include <Arduino.h>

#define PIN_ENCL_A 16
#define PIN_ENCL_B 17
#define PIN_ENCR_A 18
#define PIN_ENCR_B 19
#define TURNS 10

volatile long encL = 0, encR = 0;
volatile uint8_t stL = 0, stR = 0;
const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR isrL() {
  uint8_t cur = (digitalRead(PIN_ENCL_A) << 1) | digitalRead(PIN_ENCL_B);
  encL += QEM[(stL << 2) | cur]; stL = cur;
}
void IRAM_ATTR isrR() {
  uint8_t cur = (digitalRead(PIN_ENCR_A) << 1) | digitalRead(PIN_ENCR_B);
  encR += QEM[(stR << 2) | cur]; stR = cur;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENCL_A, INPUT_PULLUP); pinMode(PIN_ENCL_B, INPUT_PULLUP);
  pinMode(PIN_ENCR_A, INPUT_PULLUP); pinMode(PIN_ENCR_B, INPUT_PULLUP);
  attachInterrupt(PIN_ENCL_A, isrL, CHANGE); attachInterrupt(PIN_ENCL_B, isrL, CHANGE);
  attachInterrupt(PIN_ENCR_A, isrR, CHANGE); attachInterrupt(PIN_ENCR_B, isrR, CHANGE);
  Serial.println("Type 0 to zero, turn both wheels 10 times, then type c");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') { encL = 0; encR = 0; Serial.println("zeroed - turn the wheels now"); }
    if (c == 'c') {
      float l = labs(encL) / (float)TURNS, r = labs(encR) / (float)TURNS;
      Serial.printf("LEFT  %.0f counts/rev\nRIGHT %.0f counts/rev\nUSE   #define COUNTS_PER_REV %.0f.0f\n", l, r, (l + r) / 2);
    }
  }
  static uint32_t t = 0;
  if (millis() - t > 500) { t = millis(); Serial.printf("live: L=%ld R=%ld\n", labs(encL), labs(encR)); }
}
