/*
  PERIGEE mount firmware  (Arduino Uno on the rotating head)
  ------------------------------------------------------------
  Two goBILDA Stingray servo gearboxes in feedback mode:
    AZ  Stingray-4  3215-0001-0004   450 deg of travel   signal D9   feedback A0
    EL  Stingray-9  3215-0001-0009   200 deg of travel   signal D10  feedback A1
  plus an AS5600 magnetic encoder on the elevation axis (I2C, A4/A5) and the PC on the hardware serial
  (USB, or an HC-05 style Bluetooth SPP module on D0/D1 set to the same baud).

  Line protocol, ASCII, newline terminated, angles in mount-frame degrees (AZ 0..450, EL -5..185):
    PC -> mount                          mount -> PC
    PING                                 PONG
    ID                                   ID PERIGEE-MOUNT fw1 AZ 0-450 EL -5-185
    ?                                    one telemetry line
    GO az el                             OK GO az el        (ERR ... if out of range: clamped)
    AZ deg  /  EL deg                    OK AZ deg / OK EL deg
    STOP                                 OK STOP            (hold where the axes are now)
    PARK                                 OK PARK az el
    RATE az_dps el_dps                   OK RATE ...        (slew speed limits)
    TEL hz                               OK TEL hz          (telemetry rate, 0 = off)
    RAW AZ us  /  RAW EL us              OK RAW ...         (drive a raw pulse width: calibration only)
    telemetry:  T az_cmd el_cmd az_fb el_fb moving enc      az/el_fb from the feedback wires, enc = AS5600 raw
    on boot:    READY PERIGEE-MOUNT fw1

  The PC does all the sky geometry; this side only slews each axis toward its target at a limited rate
  so the servos never see a step. Positions are open loop (the pulse width IS the command on these
  servos); the feedback wires and the encoder are reported so the PC can watch the axes actually move.

  CALIBRATION (do once, with RAW and the telemetry):
    * PULSE_MIN/PULSE_MAX: pulse widths at the ends of travel for each gearbox (goBILDA: 500..2500 us
      covers the full programmed travel; adjust if the servo was programmed differently).
    * FB_MIN/FB_MAX: the analogRead() values on the feedback wire at those same ends (the wire gives a
      voltage proportional to position; measure both ends with RAW AZ 500 / RAW AZ 2500 and note enc/fb).
    * EL_OFFSET: mount elevation = gearbox angle - EL_OFFSET (5 -> the -5 deg end is at pulse min).
    * The PC's control.toml holds the one sky calibration (az_center_bearing_deg); nothing here.
*/
#include <Servo.h>
#include <Wire.h>

#define BAUD 115200
#define AZ_PIN 9
#define EL_PIN 10
#define AZ_FB_PIN A0
#define EL_FB_PIN A1

const float AZ_TRAVEL = 450.0;           // deg, Stingray-4
const float EL_TRAVEL = 200.0;           // deg, Stingray-9
const float EL_OFFSET = 5.0;             // mount el = gearbox angle - 5  (mount el range -5 .. 195, limited below)
const float EL_MIN = -5.0, EL_MAX = 185.0;
const int   AZ_PULSE_MIN = 500, AZ_PULSE_MAX = 2500;   // us at gearbox angle 0 and AZ_TRAVEL
const int   EL_PULSE_MIN = 500, EL_PULSE_MAX = 2500;   // us at gearbox angle 0 and EL_TRAVEL
const int   AZ_FB_MIN = 0, AZ_FB_MAX = 1023;           // analogRead at the two ends (measure and fill in)
const int   EL_FB_MIN = 0, EL_FB_MAX = 1023;
const float PARK_AZ = 225.0, PARK_EL = 45.0;
const unsigned long TICK_MS = 20;        // slew update period (50 Hz servo frame)

Servo az, el;
float azCmd = PARK_AZ, elCmd = PARK_EL;  // where the axes are being driven right now (ramped)
float azTgt = PARK_AZ, elTgt = PARK_EL;  // where they should end up
float azRate = 20.0, elRate = 15.0;      // deg/s
float telHz = 0.0;
bool  rawMode = false;                   // RAW takes over the pulse until the next GO/AZ/EL/STOP/PARK
unsigned long lastTick = 0, lastTel = 0;
char line[64]; uint8_t lineLen = 0;

int azPulse(float a) { return (int)(AZ_PULSE_MIN + (AZ_PULSE_MAX - AZ_PULSE_MIN) * constrain(a, 0.0, AZ_TRAVEL) / AZ_TRAVEL); }
int elPulse(float e) { float g = constrain(e + EL_OFFSET, 0.0, EL_TRAVEL); return (int)(EL_PULSE_MIN + (EL_PULSE_MAX - EL_PULSE_MIN) * g / EL_TRAVEL); }
float azFeedback() { return (float)(analogRead(AZ_FB_PIN) - AZ_FB_MIN) * AZ_TRAVEL / (float)(AZ_FB_MAX - AZ_FB_MIN); }
float elFeedback() { return (float)(analogRead(EL_FB_PIN) - EL_FB_MIN) * EL_TRAVEL / (float)(EL_FB_MAX - EL_FB_MIN) - EL_OFFSET; }

// AS5600 raw angle (0..4095), -1 when the chip does not answer
int as5600Raw() {
  Wire.beginTransmission(0x36); Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(0x36, 2) != 2) return -1;
  int hi = Wire.read(), lo = Wire.read();
  return ((hi & 0x0F) << 8) | lo;
}

void setTarget(float a, float e, bool report) {
  bool clamped = a < 0 || a > AZ_TRAVEL || e < EL_MIN || e > EL_MAX;
  azTgt = constrain(a, 0.0, AZ_TRAVEL); elTgt = constrain(e, EL_MIN, EL_MAX);
  rawMode = false;
  if (!report) return;
  if (clamped) { Serial.print(F("ERR out of range, clamped to ")); } else { Serial.print(F("OK GO ")); }
  Serial.print(azTgt, 2); Serial.print(' '); Serial.println(elTgt, 2);
}

void telemetry() {
  Serial.print(F("T ")); Serial.print(azCmd, 2); Serial.print(' '); Serial.print(elCmd, 2); Serial.print(' ');
  Serial.print(azFeedback(), 2); Serial.print(' '); Serial.print(elFeedback(), 2); Serial.print(' ');
  bool moving = fabs(azCmd - azTgt) > 0.01 || fabs(elCmd - elTgt) > 0.01;
  Serial.print(moving ? 1 : 0); Serial.print(' '); Serial.println(as5600Raw());
}

void handle(char* s) {
  // upper-case the verb, split off the numbers
  char* verb = strtok(s, " \t");
  if (!verb) return;
  for (char* p = verb; *p; ++p) *p = toupper(*p);
  char* a1 = strtok(NULL, " \t"); char* a2 = strtok(NULL, " \t");
  if (!strcmp(verb, "PING")) Serial.println(F("PONG"));
  else if (!strcmp(verb, "ID")) Serial.println(F("ID PERIGEE-MOUNT fw1 AZ 0-450 EL -5-185"));
  else if (!strcmp(verb, "?")) telemetry();
  else if (!strcmp(verb, "GO") && a1 && a2) setTarget(atof(a1), atof(a2), true);
  else if (!strcmp(verb, "AZ") && a1) { setTarget(atof(a1), elTgt, false); Serial.print(F("OK AZ ")); Serial.println(azTgt, 2); }
  else if (!strcmp(verb, "EL") && a1) { setTarget(azTgt, atof(a1), false); Serial.print(F("OK EL ")); Serial.println(elTgt, 2); }
  else if (!strcmp(verb, "STOP")) { azTgt = azCmd; elTgt = elCmd; rawMode = false; Serial.println(F("OK STOP")); }
  else if (!strcmp(verb, "PARK")) { setTarget(PARK_AZ, PARK_EL, false); Serial.print(F("OK PARK ")); Serial.print(PARK_AZ, 1); Serial.print(' '); Serial.println(PARK_EL, 1); }
  else if (!strcmp(verb, "RATE") && a1 && a2) { azRate = constrain(atof(a1), 0.1, 90.0); elRate = constrain(atof(a2), 0.1, 90.0); Serial.print(F("OK RATE ")); Serial.print(azRate, 1); Serial.print(' '); Serial.println(elRate, 1); }
  else if (!strcmp(verb, "TEL") && a1) { telHz = constrain(atof(a1), 0.0, 20.0); Serial.print(F("OK TEL ")); Serial.println(telHz, 1); }
  else if (!strcmp(verb, "RAW") && a1 && a2) {
    for (char* p = a1; *p; ++p) *p = toupper(*p);
    int us = constrain(atoi(a2), 400, 2600); rawMode = true;
    if (!strcmp(a1, "AZ")) { az.writeMicroseconds(us); Serial.print(F("OK RAW AZ ")); Serial.println(us); }
    else if (!strcmp(a1, "EL")) { el.writeMicroseconds(us); Serial.print(F("OK RAW EL ")); Serial.println(us); }
    else Serial.println(F("ERR RAW needs AZ or EL"));
  }
  else { Serial.print(F("ERR unknown command ")); Serial.println(verb); }
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  az.attach(AZ_PIN, 400, 2600); el.attach(EL_PIN, 400, 2600);
  az.writeMicroseconds(azPulse(azCmd)); el.writeMicroseconds(elPulse(elCmd));
  Serial.println(F("READY PERIGEE-MOUNT fw1"));
}

void loop() {
  // serial: assemble lines
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (lineLen) { line[lineLen] = 0; handle(line); lineLen = 0; } }
    else if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
  }
  unsigned long now = millis();
  // slew: move each command toward its target at most rate*dt per tick, then write the pulse
  if (now - lastTick >= TICK_MS) {
    float dt = (now - lastTick) / 1000.0; lastTick = now;
    if (!rawMode) {
      float da = azTgt - azCmd, ma = azRate * dt; azCmd += constrain(da, -ma, ma);
      float de = elTgt - elCmd, me = elRate * dt; elCmd += constrain(de, -me, me);
      az.writeMicroseconds(azPulse(azCmd)); el.writeMicroseconds(elPulse(elCmd));
    }
  }
  if (telHz > 0 && now - lastTel >= (unsigned long)(1000.0 / telHz)) { lastTel = now; telemetry(); }
}
