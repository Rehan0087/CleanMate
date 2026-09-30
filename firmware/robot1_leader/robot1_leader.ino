// CleanMate - Robot 1 (Leader) firmware v2.3

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if __has_include(<esp_mac.h>)
  #include <esp_mac.h>
#endif
#include <VL53L0X.h>
#include <Preferences.h>

static const uint8_t LEADER_MAC[6]   = {0xEC, 0xE3, 0x34, 0x22, 0x25, 0x50};
static const uint8_t FOLLOWER_MAC[6] = {0xF0, 0x24, 0xF9, 0x0E, 0x2B, 0xE4};
#define WIFI_CHANNEL 1

#define PIN_PWMA 25
#define PIN_AIN1 26
#define PIN_AIN2 27
#define PIN_PWMB 14
#define PIN_BIN1 33
#define PIN_BIN2 32
#define PIN_STBY 13
#define PIN_ENCL_A 16
#define PIN_ENCL_B 17
#define PIN_ENCR_A 18
#define PIN_ENCR_B 19
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_TRIG_FL 5
#define PIN_ECHO_FL 23
#define PIN_TRIG_FR 4
#define PIN_ECHO_FR 35
#define USE_BATT_SENSE 0
#define PIN_BATT 35
#define PIN_LED 2

#define USE_SONAR 1
#define SONAR_STOP_MM 200

#define MOTOR_L_INV 1
#define MOTOR_R_INV 0
#define ENC_L_INV 0
#define ENC_R_INV 1
#define IMU_YAW_SIGN 1.0f

#define WHEEL_DIA_M 0.043f
#define TRACK_M 0.17f
#define COUNTS_PER_REV 10850.0f
static const float M_PER_COUNT = (PI * WHEEL_DIA_M) / COUNTS_PER_REV;

#define KP_V 155.0f
#define KI_V 39.0f
#define FF_L 1030.0f
#define FF_R 980.0f
#define ACCEL_MPS2 0.12f

#define KH_P 0.006f
#define KH_I 0.001f
#define KH_D 0.0008f
#define H_TRIM_MAX 0.05f
#define HOLD_KP 3.0f
#define HOLD_KD 0.4f
#define HOLD_PWM_MIN 55.0f
#define HOLD_PWM_MAX 140.0f
#define HOLD_DEADBAND_DEG 2.0f
#define TURN_RATE_DPS 25.0f
#define BODY_W_MM 110.0f
#define BW_LEARN_RATE 0.6f
#define STEP_PAUSE_MS 500
#define TURN_SETTLE_DEG 3.0f
#define MISSION_V_DEFAULT 0.08f
#define KARC 1.5f
#define FF_B_DEFAULT 25.0f

#define LEADER_ON_LEFT 0
#define KY_DEG_PER_MM 0.08f
#define BIAS_MAX_DEG 10.0f
#define KX_MPS_PER_M 1.0f
#define GAP_TARGET_DEFAULT_MM 170.0f
#define GAP_OFFSET_MM 0.0f
#define GAP_JUMP_MM 40.0f
#define GAP_JUMP_CONFIRM 4
#define GAP_DAMP_S 2.0f
#define GAP_STOP_NEAR_MM 70.0f
#define KY_NEAR_DEG_PER_MM 0.15f
#define BIAS_NEAR_MAX_DEG 20.0f
#define LEADER_REPEL_MARGIN_MM 20.0f
#define LEADER_REPEL_DEG_PER_MM 0.06f
#define LEADER_REPEL_MAX_DEG 8.0f
#define GAP_STOP_FAR_MM 350.0f
#define GAP_LOST_MM 350.0f
#define GAP_FOUND_MM 300.0f
#define SEARCH_SPAN_M 0.10f
#define SEARCH_V 0.05f
#define SEARCH_SWEEPS 3
#define SEARCH_DEBOUNCE_MS 300
#define SEARCH_TIMEOUT_MS 15000
#define IMU_FAIL_TICKS 50
#define V_MAX_CMD 0.14f
#define RJ_PAIRED_MS 1000
#define RJ_LOST_MS 1500
#define RJ_STILL_MS 1000
#define RJ_V 0.05f
#define RJ_CENTER_V 0.03f
#define RJ_EDGE_LAG_M 0.004f
#define RJ_SCAN_MAX_M 0.25f
#define HOLD_GIVEUP_MS 4000
#define LONG_STOP_M 0.15f

#define COMMS_TIMEOUT_MS 350
#define LINK_TIMEOUT_MS 1000
#define BATT_LOW_MV 9600
#define BATT_PRESENT_MV 6000

enum Role  { ROLE_NONE = 0, ROLE_LEADER = 1, ROLE_FOLLOWER = 2 };
enum State { ST_IDLE = 0, ST_RUN = 1, ST_PAUSE = 2, ST_FAULT = 3 };
enum Mode  { MODE_STOP = 0, MODE_DRIVE = 1, MODE_PIVOT = 2, MODE_RESYNC = 3, MODE_RESUME = 4 };
enum Fault { F_NONE = 0, F_COMMS, F_GAP_NEAR, F_GAP_FAR, F_LONG, F_OBST, F_BATT, F_STALL, F_PARTNER, F_LINK };
static const char* FAULT_NAME[] = {"none", "comms-lost", "gap-too-near", "gap-lost/far", "long-error",
                                   "obstacle", "low-battery", "stall/encoder", "partner-fault", "link-lost"};

typedef struct __attribute__((packed)) {
  uint8_t robot_id; uint8_t state; uint32_t seq; uint32_t t_ms;
  int32_t enc_left; int32_t enc_right;
  float heading_deg; float dist_travelled_m; float v_actual_mps;
  uint16_t lateral_gap_mm; uint16_t dist_fl_mm; uint16_t dist_fr_mm; uint16_t batt_mv;
  uint8_t flags;
} RobotTelemetry;

typedef struct __attribute__((packed)) {
  uint8_t target_id; uint8_t mode; uint32_t seq;
  float v_lin_mps; float w_ang_rps; float target_gap_mm;
  uint8_t cmd_flags;
} CommandPacket;

typedef struct __attribute__((packed)) { char txt[25]; } TextCmd;
static const uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Role role = ROLE_NONE;
State state = ST_IDLE;
Fault fault = F_NONE;
const uint8_t* peerMac = nullptr;

volatile int32_t encLraw = 0, encRraw = 0;
volatile uint8_t stL = 0, stR = 0;
static const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
int32_t lastEL = 0, lastER = 0;

float vL = 0, vR = 0, distM = 0, yawEnc = 0, yawImu = 0, yaw = 0, yawRate = 0, prevYawEnc = 0;
float gzBias = 0;
bool imuOk = false; float yawSign = IMU_YAW_SIGN; bool holdMode = false;
bool autoHold = true, holdSuspended = false; float holdYaw = 0; uint32_t holdBusySince = 0;
bool searchActive = false; uint8_t searchPhase = 0, searchSweeps = 0; float searchStart = 0, searchDir = 1;
uint32_t gapLostSince = 0, searchT0 = 0; int imuFail = 0;
bool autoRejoin = true, paired = false, rjActive = false; int rjIdx = 0; float rjStart = 0, rjV = 0;
uint8_t rjStage = 0; float rjDir = 1, rjEdge1 = 0, rjMid = 0; int rjMiss = 0;
uint32_t pairSince = 0, rjLostSince = 0, stillSince = 0, rjCmdMs = 0; float rjCmdV = 0;
static const float RJ_STEPS[] = {0.05f, -0.05f, 0.10f, -0.10f, 0.15f, -0.15f, 0.0f};
static const int RJ_NSTEPS = sizeof(RJ_STEPS) / sizeof(RJ_STEPS[0]);
volatile uint32_t trigUs[2] = {0, 0}; bool sonarNew = false;
enum StepType : uint8_t { STEP_FWD = 0, STEP_TURN = 1 };
struct Step { uint8_t type; float val; };
Step mission[48]; int missionLen = 0, missionIdx = 0;
bool missionActive = false; uint8_t stepPhase = 0; uint32_t phaseT0 = 0;
float hdgSp = 0, stepStartHdg = 0, missionV = MISSION_V_DEFAULT, followerVCmd = 0;
float pivotSpacingF = 0, pivotStartHdg = 0, arcErr = 0, bodyEff = BODY_W_MM, turnSpacing = 0, turnGap0 = 0;
bool pivotMode = false; float hdgCmd = 0; uint32_t distZeroUntil = 0, graceUntil = 0;

float vGoal = 0, vRamp = 0, vBase = 0;
float intL = 0, intR = 0, hInt = 0;
float gapTarget = GAP_TARGET_DEFAULT_MM;
bool manual = false, localTest = false;
int manL = 0, manR = 0;
uint32_t zeroHoldUntil = 0;
int stallTicks = 0;
float pwmLout = 0, pwmRout = 0;

Preferences prefs;
float distScale = 1.0f;
float ffKL = FF_L, ffKR = FF_R;
float ffBL = 0, ffBR = 0;
int32_t encZL = 0, encZR = 0; float wheelDiaMm = WHEEL_DIA_M * 1000.0f;
float calTarget = 0, lastCalEnc = 0; bool calDriving = false, calPending = false; uint32_t calStopMs = 0;
int ffStage = -1; uint32_t ffT0 = 0; int32_t ffEL0 = 0, ffER0 = 0;
static const int FF_LVL[3] = {70, 100, 130};
float ffVL[3], ffVR[3];

VL53L0X tof;
bool tofOk = false, gapValid = false;
uint16_t tofRaw = 0; uint32_t tofReads = 0;
float gapMm = 0;
uint32_t tofMs = 0;
int tofBad = 0;
uint16_t gh[3] = {0, 0, 0}; uint32_t ghN = 0; int gapJump = 0; float gapRate = 0;
float battMv = 0;
volatile uint32_t echoStart[2] = {0, 0}, echoWidth[2] = {0, 0};
volatile bool echoDone[2] = {false, false};
uint16_t sonarMm[2] = {9999, 9999};
uint32_t sonarTrigMs = 0;
int sonarTurn = 0, obstacleCount = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
CommandPacket rxCmd; volatile bool rxCmdNew = false; uint32_t cmdMs = 0;
RobotTelemetry rxTel;  volatile bool rxTelNew = false; uint32_t telMs = 0;
TextCmd rxText; volatile bool rxTextNew = false;
RobotTelemetry partnerTel; bool partnerSeen = false;
RobotTelemetry leaderTel;
uint32_t txSeq = 0, lastTxMs = 0, lastTickUs = 0, lastPrintMs = 0;
bool streamStatus = false;
uint32_t lastHbMs = 0; bool remoteRun = false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  void pwmInit(int pin, int ch) { ledcAttach(pin, 20000, 8); }
  void pwmWrite(int pin, int ch, int d) { ledcWrite(pin, d); }
#else
  void pwmInit(int pin, int ch) { ledcSetup(ch, 20000, 8); ledcAttachPin(pin, ch); }
  void pwmWrite(int pin, int ch, int d) { ledcWrite(ch, d); }
#endif

void motorsEnable(bool on) { digitalWrite(PIN_STBY, on ? HIGH : LOW); }

void setMotor(int pwmPin, int ch, int in1, int in2, float pwm, bool inv) {
  if (inv) pwm = -pwm;
  int d = (int)constrain(fabsf(pwm), 0, 255);
  if (pwm > 0.5f)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (pwm < -0.5f){ digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                 { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); d = 0; }
  pwmWrite(pwmPin, ch, d);
}
void driveMotors(float l, float r) {
  setMotor(PIN_PWMA, 0, PIN_AIN1, PIN_AIN2, l, MOTOR_L_INV);
  setMotor(PIN_PWMB, 1, PIN_BIN1, PIN_BIN2, r, MOTOR_R_INV);
}
void stopMotors() { driveMotors(0, 0); motorsEnable(false); pwmLout = pwmRout = 0; intL = intR = hInt = 0; }

void IRAM_ATTR isrL() {
  uint8_t cur = (digitalRead(PIN_ENCL_A) << 1) | digitalRead(PIN_ENCL_B);
  encLraw += QEM[(stL << 2) | cur]; stL = cur;
}
void IRAM_ATTR isrR() {
  uint8_t cur = (digitalRead(PIN_ENCR_A) << 1) | digitalRead(PIN_ENCR_B);
  encRraw += QEM[(stR << 2) | cur]; stR = cur;
}
void IRAM_ATTR echoEdge(int i, int pin) {
  uint32_t t = micros();
  if (digitalRead(pin)) { echoStart[i] = (t - trigUs[i] < 12000) ? t : 0; }
  else if (echoStart[i]) { echoWidth[i] = t - echoStart[i]; echoStart[i] = 0; echoDone[i] = true; }
}
void IRAM_ATTR isrEcho0() { echoEdge(0, PIN_ECHO_FL); }
void IRAM_ATTR isrEcho1() { echoEdge(1, PIN_ECHO_FR); }

uint8_t imuAddr = 0x68;
void i2cScan() {
  Serial.print("I2C scan:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; } }
  Serial.println(n ? "   (0x68/0x69 = MPU6050, 0x29 = VL53L0X)" : " nothing found - check SDA/SCL/3V3/GND");
}
bool imuInitAt(uint8_t a) {
  Wire.beginTransmission(a); Wire.write(0x6B); Wire.write(0x00); if (Wire.endTransmission()) return false;
  delay(10);
  Wire.beginTransmission(a); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(a); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  imuAddr = a; return true;
}
bool imuInit() {
  for (int tries = 0; tries < 5; tries++) {
    if (imuInitAt(0x68) || imuInitAt(0x69)) { Serial.printf("MPU6050 found at 0x%02X\n", imuAddr); return true; }
    delay(50);
  }
  return false;
}
bool readGz(float &dps) {
  Wire.beginTransmission(imuAddr); Wire.write(0x47);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(imuAddr, (uint8_t)2) != 2) return false;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  dps = raw / 65.5f;
  return true;
}
void imuCalibrate() {
  Serial.println("IMU: keep robot still, calibrating gyro bias...");
  float sum = 0; int n = 0;
  for (int i = 0; i < 400; i++) { float g; if (readGz(g)) { sum += g; n++; } delay(3); }
  gzBias = n ? sum / n : 0;
  Serial.printf("IMU bias = %.3f dps (%d samples)\n", gzBias, n);
}

void readToF() {
  if (!tofOk) { gapValid = false; return; }
  if (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    tofMs = millis(); tofRaw = r; tofReads++;
    if (r > 0 && r < 1200) {
      gh[ghN % 3] = r; ghN++;
      float med = (ghN < 3) ? r : (float)max(min(gh[0], gh[1]), min(max(gh[0], gh[1]), gh[2]));
      float m = med + GAP_OFFSET_MM;
      tofBad = 0;
      if (gapValid && fabsf(m - gapMm) > GAP_JUMP_MM && ++gapJump < GAP_JUMP_CONFIRM) {
      } else {
        float prev = gapMm;
        gapMm = (!gapValid || gapJump >= GAP_JUMP_CONFIRM) ? m : 0.6f * gapMm + 0.4f * m;
        if (gapValid) gapRate += 0.15f * ((gapMm - prev) / 0.02f - gapRate);
        else gapRate = 0;
        gapJump = 0; gapValid = true;
      }
    } else if (++tofBad > 5) { gapValid = false; ghN = 0; gapJump = 0; gapRate = 0; }
  }
  if (millis() - tofMs > 150) gapValid = false;
}

void readBattery() {
#if !USE_BATT_SENSE
  battMv = 0; return;
#endif
  float mv = analogReadMilliVolts(PIN_BATT) * (13.3f / 3.3f);
  battMv = (battMv == 0) ? mv : 0.95f * battMv + 0.05f * mv;
}

void sonarUpdate() {
#if USE_SONAR
  for (int i = 0; i < 2; i++) if (echoDone[i]) {
    echoDone[i] = false;
    uint32_t w = echoWidth[i];
    sonarMm[i] = (w < 100 || w > 25000) ? 9999 : (uint16_t)(w * 0.1715f);
    sonarNew = true;
  }
  if (millis() - sonarTrigMs >= 30) {
    if (!echoDone[sonarTurn] && echoStart[sonarTurn] == 0) { sonarMm[sonarTurn] = 9999; sonarNew = true; }
    sonarTurn ^= 1; sonarTrigMs = millis();
    int pin = sonarTurn ? PIN_TRIG_FR : PIN_TRIG_FL;
    echoStart[sonarTurn] = 0;
    digitalWrite(pin, HIGH); delayMicroseconds(10); digitalWrite(pin, LOW);
    trigUs[sonarTurn] = micros();
  }
#endif
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  portENTER_CRITICAL(&mux);
  if (len == sizeof(CommandPacket))        { memcpy(&rxCmd, data, len); rxCmdNew = true; }
  else if (len == sizeof(RobotTelemetry))  { memcpy(&rxTel, data, len); rxTelNew = true; }
  else if (len == sizeof(TextCmd))         { memcpy(&rxText, data, len); rxText.txt[24] = 0; rxTextNew = true; }
  portEXIT_CRITICAL(&mux);
}

void sendComms() {
  uint32_t now = millis();
  RobotTelemetry t = {};
  t.robot_id = role; t.state = state; t.seq = txSeq; t.t_ms = now;
  t.enc_left = lastEL; t.enc_right = lastER;
  t.heading_deg = yaw; t.dist_travelled_m = distM; t.v_actual_mps = (vL + vR) * 0.5f;
  t.lateral_gap_mm = gapValid ? (uint16_t)gapMm : 0;
  t.dist_fl_mm = sonarMm[0]; t.dist_fr_mm = sonarMm[1]; t.batt_mv = (uint16_t)battMv;
  t.flags = (obstacleCount > 0) | ((battMv > BATT_PRESENT_MV && battMv < BATT_LOW_MV) << 1) | ((fault == F_STALL) << 2)
            | (((state == ST_FAULT) ? (uint8_t)fault : 0) << 3)
            | (((searchActive || rjActive) ? 1 : 0) << 7);
  esp_now_send(BCAST_MAC, (uint8_t*)&t, sizeof(t));

  if (role == ROLE_FOLLOWER && rjActive) {
    CommandPacket c = {};
    c.target_id = 1; c.seq = txSeq; c.mode = MODE_RESYNC; c.v_lin_mps = -rjV;
    esp_now_send(peerMac, (uint8_t*)&c, sizeof(c));
  }
  if (role == ROLE_LEADER) {
    CommandPacket c = {};
    c.target_id = 2; c.seq = txSeq;
    c.mode = (state == ST_RUN && !localTest) ? (pivotMode ? MODE_PIVOT : MODE_DRIVE) : MODE_STOP;
    c.v_lin_mps = pivotMode ? stepStartHdg : vRamp;
    c.w_ang_rps = hdgSp;
    c.target_gap_mm = pivotMode ? turnSpacing : gapTarget;
    c.cmd_flags = ((state == ST_FAULT) ? 1 : 0) | ((now < zeroHoldUntil) ? 2 : 0) | ((now < distZeroUntil) ? 4 : 0);
    esp_now_send(peerMac, (uint8_t*)&c, sizeof(c));
  }
  txSeq++;
}

void commsInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init FAILED"); return; }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, peerMac, 6); p.channel = WIFI_CHANNEL; p.encrypt = false;
  esp_now_add_peer(&p);
  esp_now_peer_info_t b = {};
  memcpy(b.peer_addr, BCAST_MAC, 6); b.channel = WIFI_CHANNEL; b.encrypt = false;
  esp_now_add_peer(&b);
}

void startSearch(const char* why) {
  searchActive = true; searchPhase = 0; searchSweeps = 0; searchStart = distM; searchT0 = millis();
  searchDir = (leaderTel.dist_travelled_m >= distM) ? 1.0f : -1.0f;
  Serial.printf("SEARCH (%s): sweeping +-10 cm, %s first\n", why, searchDir > 0 ? "forward" : "backward");
}
void enterFault(Fault f) {
  if (state == ST_FAULT) return;
  remoteRun = false; calDriving = false; calPending = false; ffStage = -1; holdMode = false; missionActive = false; pivotMode = false; followerVCmd = 0; searchActive = false;
  holdYaw = yaw; holdBusySince = 0;
  fault = f; state = ST_FAULT; manual = false; vGoal = vRamp = vBase = 0;
  stopMotors();
  Serial.printf("FAULT: %s\n", FAULT_NAME[f]);
}
void zeroRef() {
  yawImu = 0; yawEnc = 0; prevYawEnc = 0; distM = 0; hInt = 0; intL = intR = 0; hdgSp = 0; hdgCmd = 0; holdYaw = 0;
}
void stopRun() {
  remoteRun = false; calDriving = false; ffStage = -1; holdMode = false; missionActive = false; pivotMode = false; followerVCmd = 0; searchActive = false;
  holdYaw = yaw; holdBusySince = 0;
  state = ST_IDLE; manual = false; localTest = false; vGoal = vRamp = vBase = 0; stopMotors();
}
void startRun(float v) {
  if (state == ST_FAULT) { Serial.println("in FAULT - send 'r' first"); return; }
  manual = false; zeroRef(); stallTicks = 0; holdSuspended = false; searchActive = false;
  missionActive = false; pivotMode = false; calDriving = false; ffStage = -1; holdMode = false; rjActive = false;
  v = constrain(v, 0.0f, V_MAX_CMD);
  vGoal = v; vRamp = 0; state = ST_RUN;
  motorsEnable(true);
  if (role == ROLE_LEADER) { zeroHoldUntil = millis() + 250; localTest = false; }
  else { localTest = true; vBase = v; }
}

void calPrint();
void rejoinStop(const char* why);
bool startMission(int n);
int addUturn(int k, float pitch);
void printStatus() {
  if (role == ROLE_FOLLOWER) Serial.printf("ToF: found=%d reads=%lu lastRaw=%u mm\n", tofOk, (unsigned long)tofReads, tofRaw);
  calPrint();
  if (!imuOk) Serial.println("IMU: NOT FOUND (heading from encoders)");
  if (missionActive) Serial.printf("MISSION step %d/%d phase %d hdgSp=%.1f\n", missionIdx + 1, missionLen, stepPhase, hdgSp);
  Serial.printf("SONAR: L=%u R=%u mm%s%s%s\n", sonarMm[0], sonarMm[1], searchActive ? "   SEARCHING" : "", rjActive ? "   REJOIN" : "", paired ? "   paired" : "");
  if (pivotMode) Serial.printf("PIVOT: hdgCmd=%.1f start=%.1f arcErr=%.3f\n", hdgCmd, pivotStartHdg, arcErr);
  Serial.printf("[%s] st=%d fault=%s v=%.3f/%.3f dist=%.3f yaw=%.1f gap=%s%.0f tgt=%.0f batt=%.0f pwm=%.0f/%.0f\n",
    role == ROLE_LEADER ? "LEAD" : "FOLL", state, FAULT_NAME[fault], vL, vR, distM, yaw,
    gapValid ? "" : "!", gapMm, gapTarget, battMv, pwmLout, pwmRout);
}

void calLoad() {
  prefs.begin("cmcal2", true);
  distScale = prefs.getFloat("ds", 1.0f);
  ffKL = prefs.getFloat("kl", FF_L); ffKR = prefs.getFloat("kr", FF_R);
  ffBL = prefs.getFloat("b2l", FF_B_DEFAULT); ffBR = prefs.getFloat("b2r", FF_B_DEFAULT);
  wheelDiaMm = prefs.getFloat("dia", WHEEL_DIA_M * 1000.0f);
  yawSign = prefs.getFloat("ys", IMU_YAW_SIGN);
  bodyEff = prefs.getFloat("bw", BODY_W_MM);
  prefs.end();
}
void calSave() {
  prefs.begin("cmcal2", false);
  prefs.putFloat("ds", distScale);
  prefs.putFloat("kl", ffKL); prefs.putFloat("kr", ffKR);
  prefs.putFloat("b2l", ffBL); prefs.putFloat("b2r", ffBR);
  prefs.putFloat("dia", wheelDiaMm);
  prefs.putFloat("ys", yawSign);
  prefs.putFloat("bw", bodyEff);
  prefs.end();
}
void calPrint() {
  Serial.printf("CAL: wheel %.1f mm, counts/rev %.0f  ", wheelDiaMm, COUNTS_PER_REV * (wheelDiaMm / (WHEEL_DIA_M * 1000.0f)) / distScale);
  Serial.printf("CAL: width %.0f mm  ", bodyEff);
  Serial.printf("CAL: distScale=%.4f  FF L=%.0f*v+%.0f  R=%.0f*v+%.0f\n", distScale, ffKL, ffBL, ffKR, ffBR);
}
void startCal(float m) {
  if (m < 0.2f || m > 3.0f) m = 1.0f;
  startRun(0.10f);
  if (state != ST_RUN) return;
  localTest = true; vBase = 0.10f;
  calTarget = m; calDriving = true; calPending = false;
  Serial.printf("CAL: driving %.2f m by encoder. Mark the start position now.\n", m);
}
void calPoll(uint32_t now) {
  if (calPending && now - calStopMs > 600) {
    calPending = false; lastCalEnc = distM;
    Serial.printf("CAL: encoder says %.3f m. Measure the real distance with a tape and send: cs <metres>\n", lastCalEnc);
  }
}
void setScale(float measured) {
  if (lastCalEnc < 0.2f) { Serial.println("CAL: run 'cal 1' first"); return; }
  float r = measured / lastCalEnc;
  if (r < 0.7f || r > 1.3f) { Serial.printf("CAL: ratio %.3f looks wrong - check the measurement\n", r); return; }
  distScale *= r; lastCalEnc = 0; calSave();
  Serial.printf("CAL: distScale -> %.4f (saved)\n", distScale);
}
void handTurns(float n) {
  long cl = labs(lastEL - encZL), cr = labs(lastER - encZR);
  if (n < 1 || cl < 100 || cr < 100) { Serial.println("HAND: send 'enc 0', turn both wheels, then 'turns 10'"); return; }
  float cprL = cl / n, cprR = cr / n, cpr = (cprL + cprR) * 0.5f;
  Serial.printf("HAND: left %.0f  right %.0f counts/rev\n", cprL, cprR);
  if (fabsf(cprL - cprR) > 0.03f * cpr) Serial.println("HAND: wheels differ by >3% - recount the turns or check an encoder");
  if (cpr < 0.5f * COUNTS_PER_REV || cpr > 2.0f * COUNTS_PER_REV) { Serial.println("HAND: result far from expected, nothing saved"); return; }
  distScale = (COUNTS_PER_REV / cpr) * (wheelDiaMm / (WHEEL_DIA_M * 1000.0f));
  calSave(); calPrint();
}
void setDia(float mm) {
  if (mm < 30 || mm > 70) { Serial.println("dia must be 30-70 mm"); return; }
  distScale *= mm / wheelDiaMm; wheelDiaMm = mm; calSave(); calPrint();
}
void startFF() {
  if (state == ST_FAULT) { Serial.println("in FAULT - send 'r' first"); return; }
  zeroRef(); stallTicks = 0; manual = true; localTest = true; state = ST_RUN; motorsEnable(true);
  ffStage = 0; ffT0 = millis();
  Serial.println("FFCAL: 3 steps x 1.5 s, drives about 1 m forward. Keep the path clear.");
}
void ffTick(uint32_t now) {
  uint32_t t = now - ffT0;
  pwmLout = pwmRout = FF_LVL[ffStage];
  driveMotors(pwmLout, pwmRout);
  if (t >= 700 && ffEL0 == INT32_MIN) { ffEL0 = lastEL; ffER0 = lastER; }
  if (t < 700) ffEL0 = INT32_MIN;
  if (t >= 1500) {
    float w = 0.8f;
    ffVL[ffStage] = (lastEL - ffEL0) * M_PER_COUNT * distScale / w;
    ffVR[ffStage] = (lastER - ffER0) * M_PER_COUNT * distScale / w;
    Serial.printf("FFCAL: pwm %d -> vL=%.3f vR=%.3f m/s\n", FF_LVL[ffStage], ffVL[ffStage], ffVR[ffStage]);
    if (++ffStage < 3) { ffT0 = now; return; }
    ffStage = -1; stopRun();
    float kb[2][2]; float *vs[2] = {ffVL, ffVR};
    for (int w2 = 0; w2 < 2; w2++) {
      float sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (int i = 0; i < 3; i++) { float x = vs[w2][i], y = FF_LVL[i]; sx += x; sy += y; sxx += x * x; sxy += x * y; }
      float den = 3 * sxx - sx * sx;
      if (den < 1e-6f || vs[w2][0] < 0.02f) { Serial.println("FFCAL: a wheel barely moved - check motor/encoder, nothing saved"); return; }
      kb[w2][0] = (3 * sxy - sx * sy) / den; kb[w2][1] = (sy - kb[w2][0] * sx) / 3;
    }
    ffKL = constrain(kb[0][0], 300.0f, 3000.0f); ffBL = constrain(kb[0][1], 0.0f, 80.0f);
    ffKR = constrain(kb[1][0], 300.0f, 3000.0f); ffBR = constrain(kb[1][1], 0.0f, 80.0f);
    calSave(); Serial.print("FFCAL done (saved). "); calPrint();
  }
}

void handleCmd(String s) {
  s.trim(); if (!s.length()) return;
  if (s.startsWith("cal"))   { String a = s.substring(3); a.trim(); startCal(a.length() ? a.toFloat() : 1.0f); return; }
  if (s.startsWith("cs "))   { setScale(s.substring(3).toFloat()); return; }
  if (s.startsWith("trim "))  {
    float pct = constrain(s.substring(5).toFloat(), -10.0f, 10.0f);
    distScale = constrain(distScale / (1.0f + pct / 100.0f), 0.7f, 1.3f); calSave();
    Serial.printf("TRIM %+.1f %% -> distScale %.4f (saved)\n", pct, distScale); return;
  }
  if (s.startsWith("ffcal")) { startFF(); return; }
  if (s.startsWith("hold"))  {
    if (state == ST_FAULT) { Serial.println("in FAULT - send 'r' first"); return; }
    if (!imuOk) { imuOk = imuInit(); if (imuOk) imuCalibrate(); }
    if (!imuOk) { Serial.println("hold needs the IMU - not found. Send 'i2c' to scan the bus."); return; }
    if (s.indexOf("off") > 0) { autoHold = false; driveMotors(0, 0); Serial.println("HOLD: off"); return; }
    autoHold = true; holdSuspended = false; holdYaw = yaw; holdBusySince = 0;
    Serial.println("HOLD: on - keeping this heading while parked. Turn me by hand, I turn back. 'hold off' / 'x' to stop."); return;
  }
  if (s.startsWith("fwd "))  { mission[0] = {STEP_FWD, constrain(s.substring(4).toFloat(), 0.05f, 5.0f)}; startMission(1); return; }
  if (s.startsWith("turn "))  {
    float d = constrain(s.substring(5).toFloat(), -180.0f, 180.0f);
    if (fabsf(d) < 5) { Serial.println("turn: give degrees, + left / - right, e.g. turn 90"); return; }
    mission[0] = {STEP_TURN, d}; startMission(1); return;
  }
  if (s.startsWith("uturn"))  { float pch = 0.75f; sscanf(s.c_str(), "uturn %f", &pch); startMission(addUturn(0, pch)); return; }
  if (s.startsWith("mow "))   {
    int lanes = 2; float len = 1.5f, pch = 0.75f;
    sscanf(s.c_str(), "mow %d %f %f", &lanes, &len, &pch);
    lanes = constrain(lanes, 1, 12); len = constrain(len, 0.2f, 5.0f);
    int k = 0;
    for (int i = 0; i < lanes; i++) {
      mission[k++] = {STEP_FWD, len};
      if (i < lanes - 1) k = addUturn(k, (i % 2 == 0) ? pch : -pch);
    }
    startMission(k); return;
  }
  if (s.startsWith("mv "))    { missionV = constrain(s.substring(3).toFloat(), 0.03f, 0.12f); Serial.printf("mission speed %.2f m/s\n", missionV); return; }
  if (s.startsWith("search")) {
    if (role != ROLE_FOLLOWER || state != ST_RUN || localTest) { Serial.println("search: follower only, during a formation run"); return; }
    startSearch("command"); return;
  }
  if (s.startsWith("sonar")) {
    for (int i = 0; i < 2; i++) {
      int trig = i ? PIN_TRIG_FR : PIN_TRIG_FL, echo = i ? PIN_ECHO_FR : PIN_ECHO_FL;
      delay(40);
      Serial.printf("SONAR %s: echo pin idle=%d  ", i ? "RIGHT" : "LEFT", digitalRead(echo));
      digitalWrite(trig, LOW); delayMicroseconds(4);
      digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
      uint32_t w = pulseIn(echo, HIGH, 30000);
      if (w) Serial.printf("pulse %lu us = %.0f mm\n", (unsigned long)w, w * 0.1715f);
      else Serial.println("NO PULSE - check 5V/GND to the sensor, TRIG wire, ECHO wire");
    }
    return;
  }
  if (s.startsWith("bw"))     {
    String a = s.substring(2); a.trim();
    if (a.length()) { bodyEff = constrain(a.toFloat(), 40.0f, 260.0f); calSave(); }
    Serial.printf("effective width %.0f mm (centre spacing = gap + this)\n", bodyEff); return;
  }
  if (s.startsWith("rejoin"))  {
    autoRejoin = s.indexOf("off") < 0; if (!autoRejoin && rjActive) rejoinStop("turned off");
    Serial.printf("REJOIN: %s\n", autoRejoin ? "on" : "off"); return;
  }
  if (s.startsWith("i2c"))   { i2cScan(); Serial.printf("IMU %s\n", imuOk ? "OK" : "NOT FOUND"); return; }
  if (s.startsWith("yflip")) { yawSign = -yawSign; calSave(); Serial.printf("yaw sign now %+.0f (saved)\n", yawSign); return; }
  if (s.startsWith("enc"))   {
    if (s.indexOf('0') > 0) { encZL = lastEL; encZR = lastER; Serial.println("HAND: counters zeroed - turn both wheels forward now"); }
    else Serial.printf("HAND: left %ld  right %ld counts\n", (long)labs(lastEL - encZL), (long)labs(lastER - encZR));
    return;
  }
  if (s.startsWith("turns ")) { handTurns(s.substring(6).toFloat()); return; }
  if (s.startsWith("dia "))   { setDia(s.substring(4).toFloat()); return; }
  if (s.startsWith("cfg"))   {
    if (s.indexOf("reset") > 0) { distScale = 1.0f; ffKL = FF_L; ffKR = FF_R; ffBL = ffBR = FF_B_DEFAULT; calSave(); }
    calPrint(); return;
  }
  char c = s[0]; String a = s.substring(1); a.trim();
  if (c == 'x') { stopRun(); holdSuspended = true; driveMotors(0, 0); Serial.println("E-STOP: motors off (hold paused until next start or 'hold')"); return; }
  if (c == 'd') startRun(a.toFloat());
  else if (c == 's') { stopRun(); Serial.println("stopped"); }
  else if (c == 'r') { fault = F_NONE; stopRun(); zeroRef(); Serial.println("reset"); }
  else if (c == 'z') { zeroRef(); Serial.println("zeroed"); }
  else if (c == 'g') { gapTarget = constrain(a.toFloat(), 120.0f, 250.0f); Serial.printf("gap target %.0f mm\n", gapTarget); }
  else if (c == 'm') {
    int sp = a.indexOf(' ');
    manL = a.substring(0, sp).toInt(); manR = (sp > 0) ? a.substring(sp + 1).toInt() : manL;
    if (state != ST_FAULT) { manual = true; state = ST_RUN; localTest = true; motorsEnable(true); }
  }
  else if (c == 'p') { streamStatus = !streamStatus; }
  else if (c == 'k') printStatus();
  else Serial.println("cmds: d v | s | r | z | g mm | m l r | p | k | cal m | cs m | ffcal | cfg");
}
void serialPoll() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCmd(buf); buf = ""; }
    else if (buf.length() < 40) buf += ch;
  }
}

float wheelPwm(float target, float meas, float ff, float fb, float &integ, float dt) {
  if (fabsf(target) < 0.001f) { integ = 0; return 0; }
  float e = target - meas;
  integ += e * dt;
  float lim = 200.0f / KI_V;
  integ = constrain(integ, -lim, lim);
  return constrain(ff * target + (target > 0 ? fb : -fb) + KP_V * e + KI_V * integ, -255.0f, 255.0f);
}
float headingTrim(float sp, float meas, float rate, float dt) {
  float e = sp - meas;
  hInt = constrain(hInt + e * dt, -20.0f, 20.0f);
  float t = KH_P * e + KH_I * hInt - KH_D * rate;
  return constrain(t, -H_TRIM_MAX, H_TRIM_MAX);
}

void processRx(uint32_t now) {
  CommandPacket c; RobotTelemetry t; TextCmd tx; bool nc = false, nt = false, nx = false;
  portENTER_CRITICAL(&mux);
  if (rxCmdNew) { c = rxCmd; rxCmdNew = false; nc = true; }
  if (rxTelNew) { t = rxTel; rxTelNew = false; nt = true; }
  if (rxTextNew) { tx = rxText; rxTextNew = false; nx = true; }
  portEXIT_CRITICAL(&mux);

  if (nx) {
    char c0 = tx.txt[0];
    if ((c0 == '1' || c0 == '2') && tx.txt[1] == ' ') {
      if (c0 - '0' == role) handleCmd(String(tx.txt + 2));
      if (role == ROLE_LEADER) lastHbMs = now;
    } else if (role == ROLE_LEADER) {
      lastHbMs = now;
      if (strcmp(tx.txt, "h") != 0) {
        handleCmd(String(tx.txt));
        if ((c0 == 'd' || c0 == 'm' || missionActive) && state == ST_RUN) remoteRun = true;
      }
    } else if (c0 == 's' || c0 == 'r' || c0 == 'x') handleCmd(String(tx.txt));
  }
  if (nt) {
    if (role == ROLE_LEADER && t.robot_id == 2) { partnerTel = t; partnerSeen = true; telMs = now; }
    if (role == ROLE_FOLLOWER && t.robot_id == 1) { leaderTel = t; telMs = now; }
  }
  if (nc && role == ROLE_LEADER && c.mode == MODE_RESYNC) { rjCmdV = c.v_lin_mps; rjCmdMs = now; }
  if (nc && role == ROLE_FOLLOWER) {
    cmdMs = now;
    if (c.cmd_flags & 1) { if (state == ST_RUN) stopRun(); }
    else if ((c.mode == MODE_DRIVE || c.mode == MODE_PIVOT) && state != ST_FAULT) {
      if (state != ST_RUN || localTest || manual || holdMode) { state = ST_RUN; manual = false; localTest = false; holdMode = false; motorsEnable(true); stallTicks = 0; }
      hdgCmd = c.w_ang_rps;
      pivotMode = (c.mode == MODE_PIVOT);
      if (pivotMode) pivotSpacingF = c.target_gap_mm; else gapTarget = c.target_gap_mm;
      if (pivotMode) { pivotStartHdg = c.v_lin_mps; vBase = 0; } else vBase = c.v_lin_mps;
      if (pivotMode) graceUntil = now + 800;
      if (c.cmd_flags & 2) zeroRef();
      if (c.cmd_flags & 4) { distM = 0; graceUntil = now + 800; }
    } else if (c.mode == MODE_STOP && state == ST_RUN && !localTest && !manual) stopRun();
  }
}

void safetyChecks(uint32_t now) {
  if (state != ST_RUN) return;
  if (role == ROLE_LEADER && remoteRun && now - lastHbMs > LINK_TIMEOUT_MS) { enterFault(F_LINK); return; }
  bool formation = (role == ROLE_LEADER) ? partnerSeen : !localTest;

  if (battMv > BATT_PRESENT_MV && battMv < BATT_LOW_MV) { enterFault(F_BATT); return; }

  if (role == ROLE_LEADER && partnerSeen && !manual) {
    if (now - telMs > COMMS_TIMEOUT_MS) { enterFault(F_COMMS); return; }
    if (partnerTel.state == ST_FAULT)   { enterFault(F_PARTNER); return; }
  }
  if (role == ROLE_FOLLOWER && formation && !manual) {
    if (now - cmdMs > COMMS_TIMEOUT_MS || now - telMs > COMMS_TIMEOUT_MS) { enterFault(F_COMMS); return; }
    bool lost = !gapValid || gapMm > GAP_LOST_MM;
    if (!lost || searchActive || pivotMode || now <= graceUntil) gapLostSince = 0;
    else if (!gapLostSince) gapLostSince = now;
    else if (now - gapLostSince > SEARCH_DEBOUNCE_MS) { gapLostSince = 0; startSearch("lost the leader"); }
    if (searchActive && now - searchT0 > SEARCH_TIMEOUT_MS) { enterFault(F_GAP_FAR); return; }
    if (gapValid && gapMm < GAP_STOP_NEAR_MM) { enterFault(F_GAP_NEAR); return; }
    if (!pivotMode && !searchActive && now > graceUntil && fabsf(leaderTel.dist_travelled_m - distM) > LONG_STOP_M) { enterFault(F_LONG); return; }
  }
#if USE_SONAR
  if (sonarNew) {
    sonarNew = false;
    uint16_t m = min(sonarMm[0], sonarMm[1]);
    obstacleCount = (m > 20 && m < SONAR_STOP_MM) ? obstacleCount + 1 : 0;
    if (obstacleCount >= 3) { enterFault(F_OBST); return; }
  }
#endif
  if (!manual && (fabsf(pwmLout) > 77 || fabsf(pwmRout) > 77) && fabsf(vL) < 0.005f && fabsf(vR) < 0.005f) {
    if (++stallTicks > 60) enterFault(F_STALL);
  } else stallTicks = 0;
}

void nextStep(uint32_t now) {
  missionIdx++; stepPhase = 0; phaseT0 = now; pivotMode = false; followerVCmd = 0; vRamp = 0;
  if (missionIdx >= missionLen) {
    Serial.println("MISSION: done"); missionActive = false; stopRun();
  } else Serial.printf("MISSION: step %d/%d %s %.2f\n", missionIdx + 1, missionLen,
                       mission[missionIdx].type == STEP_FWD ? "fwd" : "turn", mission[missionIdx].val);
}
float missionTick(uint32_t now, float dt) {
  Step &st = mission[missionIdx];
  float v = 0;
  if (stepPhase == 0) {
    pivotMode = false; followerVCmd = 0; vRamp = 0;
    distM = 0; distZeroUntil = now + 150;
    if (now - phaseT0 > STEP_PAUSE_MS && now >= zeroHoldUntil) {
      stepPhase = 1; phaseT0 = now; stepStartHdg = hdgSp;
      bool g = partnerSeen && partnerTel.lateral_gap_mm > 0;
      turnGap0 = g ? partnerTel.lateral_gap_mm : 0;
      turnSpacing = (g ? turnGap0 : gapTarget) + bodyEff;
    }
    return 0;
  }
  if (st.type == STEP_FWD) {
    float rem = st.val - distM;
    float vmax = sqrtf(fmaxf(0.0f, 2.0f * ACCEL_MPS2 * rem));
    float step = ACCEL_MPS2 * dt;
    vRamp += constrain(min(missionV, vmax) - vRamp, -step, step);
    if (rem <= 0.005f || (rem < 0.02f && vRamp < 0.01f)) { nextStep(now); return 0; }
    return vRamp;
  }
  float dir = st.val > 0 ? 1.0f : -1.0f;
  float target = stepStartHdg + st.val;
  pivotMode = true;
  if (stepPhase == 1) {
    hdgSp += dir * TURN_RATE_DPS * dt;
    if ((hdgSp - target) * dir >= 0) { hdgSp = target; stepPhase = 2; phaseT0 = now; }
    float w = dir * TURN_RATE_DPS * 0.0174533f;
    float R = turnSpacing / 2000.0f;
    v = w * R * (LEADER_ON_LEFT ? -1.0f : 1.0f);
    followerVCmd = -v;
    vRamp = v;
    return v;
  }
  followerVCmd = 0; vRamp = 0;
  float sideL = LEADER_ON_LEFT ? -1.0f : 1.0f;
  float R = turnSpacing / 2000.0f;
  float arcF = -sideL * R * st.val * 0.0174533f;
  bool meOk = fabsf(yaw - hdgSp) < TURN_SETTLE_DEG && fabsf(arcErr) < 0.01f;
  bool partnerOk = !partnerSeen || (fabsf(partnerTel.heading_deg - hdgSp) < TURN_SETTLE_DEG
                                    && fabsf(partnerTel.dist_travelled_m - arcF) < 0.015f);
  if ((meOk && partnerOk) || now - phaseT0 > 3500) {
    float gEnd = partnerTel.lateral_gap_mm, th = fabsf(st.val) * 0.0174533f;
    if (turnGap0 > 0 && gEnd > 0 && fabsf(gEnd - turnGap0) < 250 && th > 0.8f) {
      float dG = gEnd - turnGap0;
      bodyEff = constrain(bodyEff - BW_LEARN_RATE * dG / (1.0f - cosf(th)), 40.0f, 260.0f);
      calSave();
      Serial.printf("TURN: gap %.0f -> %.0f mm, effective width now %.0f mm (saved)\n", turnGap0, gEnd, bodyEff);
    }
    nextStep(now);
  }
  return 0;
}
bool startMission(int n) {
  if (role != ROLE_LEADER) { Serial.println("send path commands to the LEADER (or without a robot prefix)"); return false; }
  if (state == ST_FAULT) { Serial.println("in FAULT - send 'r' first"); return false; }
  if (n <= 0) return false;
  startRun(missionV);
  if (state != ST_RUN) return false;
  missionLen = n; missionIdx = 0; stepPhase = 0; phaseT0 = millis(); missionActive = true; hdgSp = 0;
  Serial.printf("MISSION: %d steps at %.2f m/s. step 1 %s %.2f\n", n, missionV,
                mission[0].type == STEP_FWD ? "fwd" : "turn", mission[0].val);
  return true;
}
int addUturn(int k, float pitch) {
  float d = pitch >= 0 ? 90.0f : -90.0f;
  mission[k++] = {STEP_TURN, d}; mission[k++] = {STEP_FWD, fabsf(pitch)}; mission[k++] = {STEP_TURN, d};
  return k;
}

void holdTick(uint32_t now) {
  float e = holdYaw - yaw;
  float p = HOLD_KP * e - HOLD_KD * yawRate;
  float out = 0;
  if (fabsf(e) > HOLD_DEADBAND_DEG && (p > 0) == (e > 0))
    out = (p > 0 ? 1 : -1) * constrain(HOLD_PWM_MIN + fabsf(p), 0.0f, HOLD_PWM_MAX);
  if (out != 0) {
    if (!holdBusySince) holdBusySince = now;
    if (now - holdBusySince > HOLD_GIVEUP_MS) {
      holdYaw = yaw; holdBusySince = 0; out = 0;
      Serial.println("HOLD: blocked - accepting current heading");
    }
  } else holdBusySince = 0;
  if (out != 0) motorsEnable(true);
  pwmLout = -out; pwmRout = out;
  driveMotors(pwmLout, pwmRout);
}

void idleDrive(float v, float dt) {
  if (v > 0 && min(sonarMm[0], sonarMm[1]) < SONAR_STOP_MM) v = 0;
  float trim = headingTrim(holdYaw, yaw, yawRate, dt);
  motorsEnable(true);
  pwmLout = wheelPwm(v - trim, vL, ffKL, ffBL, intL, dt);
  pwmRout = wheelPwm(v + trim, vR, ffKR, ffBR, intR, dt);
  driveMotors(pwmLout, pwmRout);
}
void rejoinStop(const char* why) {
  rjActive = false; rjV = 0; intL = intR = hInt = 0; driveMotors(0, 0);
  Serial.printf("REJOIN: %s\n", why);
}
bool rejoinTick(uint32_t now, float dt) {
  bool leaderIdle = (now - telMs < 500) && leaderTel.state == ST_IDLE;
  bool canRun = autoRejoin && !holdSuspended && imuOk && tofOk && state == ST_IDLE && leaderIdle;
  bool seen = gapValid && gapMm < GAP_FOUND_MM;
  if (!canRun) { if (rjActive) rejoinStop("cancelled"); pairSince = rjLostSince = 0; return false; }

  if (rjActive && rjStage == 1) {
    bool lost = !gapValid || gapMm > GAP_FOUND_MM + 50;
    rjMiss = lost ? rjMiss + 1 : 0;
    float travelled = fabsf(distM - rjEdge1);
    if (rjMiss >= 3 || travelled > RJ_SCAN_MAX_M) {
      float edge2 = distM - rjDir * (rjMiss >= 3 ? RJ_EDGE_LAG_M : 0.0f);
      rjMid = 0.5f * (rjEdge1 + edge2); rjStage = 2;
      Serial.printf("REJOIN: plate spans %.0f mm of my travel - moving back to its middle\n", fabsf(edge2 - rjEdge1) * 1000);
      return true;
    }
    rjV = rjDir * RJ_CENTER_V; idleDrive(rjV, dt); return true;
  }
  if (rjActive && rjStage == 2) {
    float err = rjMid - distM;
    if (fabsf(err) < 0.004f) { rejoinStop("in line with the leader - stopped"); holdYaw = yaw; return false; }
    rjV = (err > 0 ? 1 : -1) * RJ_CENTER_V; idleDrive(rjV, dt); return true;
  }
  if (rjActive) {
    if (seen) {
      rjStage = 1; rjEdge1 = distM; rjDir = (rjV >= 0) ? 1.0f : -1.0f; rjMiss = 0;
      Serial.println("REJOIN: found the leader's plate edge - centring");
      return true;
    }
    float target = rjStart + RJ_STEPS[rjIdx];
    float err = target - distM;
    if (fabsf(err) < 0.01f) {
      if (++rjIdx >= RJ_NSTEPS) { rejoinStop("leader not found - giving up (put them side by side again)"); paired = false; return false; }
      return true;
    }
    rjV = (err > 0 ? 1 : -1) * RJ_V;
    idleDrive(rjV, dt);
    return true;
  }
  if (seen) { if (!pairSince) pairSince = now; if (now - pairSince > RJ_PAIRED_MS) paired = true; rjLostSince = 0; return false; }
  pairSince = 0;
  bool still = fabsf(yawRate) < 5 && fabsf(vL) < 0.005f && fabsf(vR) < 0.005f && fabsf(leaderTel.v_actual_mps) < 0.005f;
  if (!still) stillSince = 0; else if (!stillSince) stillSince = now;
  if (!paired) return false;
  if (!rjLostSince) rjLostSince = now;
  if (now - rjLostSince > RJ_LOST_MS && stillSince && now - stillSince > RJ_STILL_MS) {
    rjActive = true; rjIdx = 0; rjStart = distM; rjLostSince = 0; intL = intR = hInt = 0; rjStage = 0; rjV = 0;
    Serial.println("REJOIN: lost the leader while parked - searching (me forward / leader back, then the opposite)");
  }
  return false;
}

void controlTick(float dt) {
  uint32_t now = millis();

  int32_t eL = encLraw * (ENC_L_INV ? -1 : 1);
  int32_t eR = encRraw * (ENC_R_INV ? -1 : 1);
  float dL = (eL - lastEL) * M_PER_COUNT * distScale, dR = (eR - lastER) * M_PER_COUNT * distScale;
  lastEL = eL; lastER = eR;
  vL = 0.7f * vL + 0.3f * (dL / dt);
  vR = 0.7f * vR + 0.3f * (dR / dt);
  distM += (dL + dR) * 0.5f;
  yawEnc += (dR - dL) / TRACK_M * 57.2958f;

  float gz;
  bool gyroOk = imuOk && readGz(gz);
  if (imuOk && !gyroOk && ++imuFail > IMU_FAIL_TICKS) {
    imuOk = false; yawEnc = yaw; prevYawEnc = yaw;
    Serial.println("IMU: stopped answering - switching to wheel-encoder heading (parked hold off)");
  }
  if (gyroOk) {
    imuFail = 0;
    if (state != ST_RUN && fabsf(vL) < 0.005f && fabsf(vR) < 0.005f && fabsf(gz - gzBias) < 1.5f)
      gzBias += 0.02f * (gz - gzBias);
    yawRate = (gz - gzBias) * yawSign; yawImu += yawRate * dt;
    yaw = yawImu;
  } else if (!imuOk) {
    yawRate = 0.7f * yawRate + 0.3f * ((yawEnc - prevYawEnc) / dt);
    yaw = yawEnc;
  }
  prevYawEnc = yawEnc;
  if (role == ROLE_FOLLOWER) readToF();
  readBattery();
  sonarUpdate();
  processRx(now);
  safetyChecks(now);

  if (state != ST_RUN) {
    bool canHold = autoHold && !holdSuspended && imuOk && !calPending
                   && (state == ST_IDLE || (state == ST_FAULT && fault != F_STALL && fault != F_BATT));
    if (role == ROLE_FOLLOWER && rejoinTick(now, dt)) return;
    bool rjCmd = role == ROLE_LEADER && state == ST_IDLE && !holdSuspended && now - rjCmdMs < 250;
    if (rjCmd) { idleDrive(rjCmdV, dt); return; }
    if (role == ROLE_LEADER && now - rjCmdMs < 400) { intL = intR = hInt = 0; }
    if (canHold) holdTick(now); else driveMotors(0, 0);
    return;
  }

  if (ffStage >= 0) { ffTick(now); return; }
  if (manual) { pwmLout = manL; pwmRout = manR; driveMotors(pwmLout, pwmRout); return; }
  if (holdMode) { holdTick(now); return; }
  if (calDriving && distM >= calTarget) {
    stopRun(); calPending = true; calStopMs = now;
  }

  float v;
  bool partnerSearch = (role == ROLE_LEADER) && partnerSeen && (partnerTel.flags & 0x80);
  if (partnerSearch) {
    vRamp = 0; v = 0;
  } else if (role == ROLE_LEADER && missionActive) {
    v = missionTick(now, dt);
    if (!missionActive) return;
  } else if (role == ROLE_LEADER) {
    if (now >= zeroHoldUntil) {
      float step = ACCEL_MPS2 * dt;
      vRamp += constrain(vGoal - vRamp, -step, step);
    }
    v = vRamp;
  } else if (localTest) {
    float step = ACCEL_MPS2 * dt;
    vRamp += constrain(vGoal - vRamp, -step, step);
    v = vRamp; vBase = vRamp;
  } else v = vBase;

  float sp = (role == ROLE_LEADER) ? hdgSp : 0;
  if (role == ROLE_LEADER && !localTest && partnerSeen && !pivotMode && fabsf(v) > 0.001f
      && partnerTel.state == ST_RUN && partnerTel.lateral_gap_mm > 0) {
    float deficit = (gapTarget - LEADER_REPEL_MARGIN_MM) - partnerTel.lateral_gap_mm;
    if (deficit > 0) {
      float away = constrain(LEADER_REPEL_DEG_PER_MM * deficit, 0.0f, LEADER_REPEL_MAX_DEG);
      sp += LEADER_ON_LEFT ? away : -away;
    }
  }
  if (role == ROLE_FOLLOWER && !localTest && searchActive) {
    sp = hdgCmd;
    float rel = distM - searchStart;
    float target = searchDir * ((searchPhase == 0) ? SEARCH_SPAN_M : (searchPhase == 1) ? -SEARCH_SPAN_M : 0.0f);
    float err = target - rel;
    if (fabsf(err) < 0.01f) {
      if (searchPhase == 2) { if (++searchSweeps >= SEARCH_SWEEPS) { searchActive = false; enterFault(F_GAP_FAR); return; } }
      searchPhase = (searchPhase + 1) % 3;
    }
    v = (err > 0 ? 1 : -1) * SEARCH_V;
    if (gapValid && gapMm < GAP_FOUND_MM) {
      searchActive = false; graceUntil = now + 1500;
      Serial.printf("SEARCH: leader found (gap %.0f mm) - resuming\n", gapMm);
    }
  } else if (role == ROLE_FOLLOWER && !localTest && pivotMode) {
    sp = hdgCmd;
  } else if (role == ROLE_FOLLOWER && !localTest) {
    float eY = gapValid ? (gapMm - gapTarget) + GAP_DAMP_S * gapRate : 0.0f;
    float bias = (eY < 0) ? constrain(KY_NEAR_DEG_PER_MM * eY, -BIAS_NEAR_MAX_DEG, 0.0f)
                          : constrain(KY_DEG_PER_MM * eY, 0.0f, BIAS_MAX_DEG);
    if (!LEADER_ON_LEFT) bias = -bias;
    sp = hdgCmd + bias;
    static float vLeadF = 0;
    vLeadF += 0.3f * (leaderTel.v_actual_mps - vLeadF);
    if (vBase > 0.001f) v = constrain(vLeadF, 0.0f, vBase * 1.1f);
    float eX = leaderTel.dist_travelled_m - distM;
    float lim = 0.15f * fabsf(vBase);
    v += constrain(KX_MPS_PER_M * eX, -lim, lim);
  }

  static float spPrev = 0, spRate = 0;
  float r = (sp - spPrev) / dt; spPrev = sp;
  if (fabsf(r) > 200) r = 0;
  spRate += 0.2f * (r - spRate);
  bool turning = pivotMode || (role == ROLE_LEADER && missionActive && stepPhase > 0 && mission[missionIdx].type == STEP_TURN);
  float ffDiff = turning ? spRate * 0.0174533f * TRACK_M * 0.5f : 0;
  if (turning) {
    float side = (role == ROLE_LEADER) ? (LEADER_ON_LEFT ? -1.0f : 1.0f) : (LEADER_ON_LEFT ? 1.0f : -1.0f);
    float R = (role == ROLE_LEADER ? turnSpacing : pivotSpacingF) / 2000.0f;
    float startH = (role == ROLE_LEADER) ? stepStartHdg : pivotStartHdg;
    float arcT = side * R * (sp - startH) * 0.0174533f;
    arcErr = arcT - distM;
    v = constrain(side * R * spRate * 0.0174533f + KARC * arcErr, -0.12f, 0.12f);
  }
  bool hdgOn = fabsf(v) > 0.001f || turning;
  float trim = hdgOn ? ffDiff + headingTrim(sp, yaw, yawRate - (turning ? spRate : 0), dt) : 0;
  float tL = v - trim, tR = v + trim;
  pwmLout = wheelPwm(tL, vL, ffKL, ffBL, intL, dt);
  pwmRout = wheelPwm(tR, vR, ffKR, ffBR, intR, dt);
  driveMotors(pwmLout, pwmRout);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STBY, OUTPUT); motorsEnable(false);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pwmInit(PIN_PWMA, 0); pwmInit(PIN_PWMB, 1);
  driveMotors(0, 0);
  pinMode(PIN_LED, OUTPUT);

  uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("MAC %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  role = ROLE_LEADER; peerMac = FOLLOWER_MAC;
  if (memcmp(mac, LEADER_MAC, 6)) { Serial.println("WRONG BOARD: this sketch is for Robot 1 - Leader (EC:E3:34:22:25:50). Halting."); while (true) delay(1000); }
  Serial.println(role == ROLE_LEADER ? "ROLE: LEADER (Robot 1, IMU)" : "ROLE: FOLLOWER (Robot 2, IMU + ToF)");

  pinMode(PIN_ENCL_A, INPUT_PULLUP); pinMode(PIN_ENCL_B, INPUT_PULLUP);
  pinMode(PIN_ENCR_A, INPUT_PULLUP); pinMode(PIN_ENCR_B, INPUT_PULLUP);
  stL = (digitalRead(PIN_ENCL_A) << 1) | digitalRead(PIN_ENCL_B);
  stR = (digitalRead(PIN_ENCR_A) << 1) | digitalRead(PIN_ENCR_B);
  attachInterrupt(PIN_ENCL_A, isrL, CHANGE); attachInterrupt(PIN_ENCL_B, isrL, CHANGE);
  attachInterrupt(PIN_ENCR_A, isrR, CHANGE); attachInterrupt(PIN_ENCR_B, isrR, CHANGE);

#if USE_SONAR
  pinMode(PIN_TRIG_FL, OUTPUT); pinMode(PIN_TRIG_FR, OUTPUT);
  pinMode(PIN_ECHO_FL, INPUT); pinMode(PIN_ECHO_FR, INPUT);
  attachInterrupt(PIN_ECHO_FL, isrEcho0, CHANGE); attachInterrupt(PIN_ECHO_FR, isrEcho1, CHANGE);
#endif

#if USE_BATT_SENSE
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
#endif

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000); Wire.setTimeOut(20);
  delay(100);
  i2cScan();
  imuOk = imuInit();
  if (imuOk) { imuCalibrate(); holdYaw = 0; Serial.println("HOLD: on (parked heading hold). 'hold off' to disable"); }
  else Serial.println("MPU6050 NOT FOUND - using wheel encoders for heading");
  if (role == ROLE_FOLLOWER) {
    tof.setTimeout(100);
    tofOk = tof.init();
    if (tofOk) { tof.setMeasurementTimingBudget(20000); tof.startContinuous(); Serial.println("VL53L0X ok"); }
    else Serial.println("VL53L0X NOT FOUND - formation will fault");
  }

  calLoad(); calPrint();
  commsInit();
  lastTickUs = micros();
  Serial.println("ready. cmds: d v | s | r | z | g mm | m l r | p | k | cal m | cs m | ffcal | cfg | enc 0 | enc | turns n | dia mm | hold | yflip | fwd m | turn deg | uturn p | mow n len p | mv v");
  Serial.println("NOTE: real top speed is about 0.17 m/s - use d 0.06 to 0.12");
  Serial.println("FIRMWARE: CleanMate v2.3 (parked re-join + centring, turn-width learning, ToF spike filter, gap damping)");
}

void loop() {
  serialPoll();
  uint32_t nowUs = micros();
  if (nowUs - lastTickUs >= 10000) {
    float dt = min((nowUs - lastTickUs) * 1e-6f, 0.05f);
    lastTickUs = nowUs;
    controlTick(dt);
  }
  uint32_t now = millis();
  calPoll(now);
  if (now - lastTxMs >= 20) { lastTxMs = now; sendComms(); }
  if (streamStatus && now - lastPrintMs >= 200) { lastPrintMs = now; printStatus(); }
  if (state == ST_FAULT) digitalWrite(PIN_LED, (now / 100) & 1);
  else if (state == ST_RUN) digitalWrite(PIN_LED, HIGH);
  else digitalWrite(PIN_LED, (now / 500) & 1);
}
