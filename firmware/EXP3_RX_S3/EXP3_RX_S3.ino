/*
  EXPERIMENT 3 - RX (Aircraft side)  v1.0.0
  Board : ESP32-S3 DevKitC (Tools > Board > esp32 > ESP32S3 Dev Module)
          Upload through the port labelled COM, USB CDC On Boot: Disabled
  Author: Mohammed Isam Yousif

  Receives UDP commands from the TX (Xbox pad) and drives:
    GPIO15 -> ESC signal (yellow)      brushless motor throttle
    GPIO16 -> MG995 signal (orange)    aileron  (roll)
    GPIO17 -> SG90  signal (orange)    elevator (pitch)
    GND    -> ESC brown + servo browns (common ground)
    ESC red (5V BEC) -> servo reds ONLY (never to the ESP)

  Safety:
    - throttle forced to 1000 us when DISARMED
    - failsafe: no valid packet for 500 ms -> throttle 1000 us, servos centre
    - servo movement smoothed and travel limited (protects MG995 / BEC)
  Telemetry back to TX: echo timestamp (round-trip latency) + Wi-Fi RSSI
  Onboard RGB LED (GPIO48): red = no link / failsafe, green = link SAFE, blue = ARMED
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

const char* SSID = "ESP_MOTOR_CTRL";
const char* PASS = "motor123";
IPAddress LOCAL_IP(192, 168, 4, 2);
IPAddress GATEWAY(192, 168, 4, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress TX_IP(192, 168, 4, 1);
const uint16_t CMD_PORT = 4210;
const uint16_t TEL_PORT = 4211;

const int ESC_PIN = 15;
const int AIL_PIN = 16;
const int ELE_PIN = 17;
const int LED_PIN = 48;

const int THR_MIN = 1000, THR_MAX = 1200;
const int SRV_MIN = 1100, SRV_MID = 1500, SRV_MAX = 1900;
const unsigned long FAILSAFE_MS = 500;
const int SERVO_STEP = 12;     // max us change per 10 ms (smooth motion)
const int THR_DOWN_STEP = 4;   // gentle normal ramp-down (protects the 12V adapter)

WiFiUDP udp;
Servo esc, servoAil, servoEle;

int tgtThr = THR_MIN, tgtAil = SRV_MID, tgtEle = SRV_MID;
int curThr = THR_MIN, curAil = SRV_MID, curEle = SRV_MID;
bool armed = false, failsafe = true;
unsigned long lastValidMs = 0, lastStep = 0, lastLog = 0;
uint32_t goodPackets = 0, badPackets = 0;

void setLed(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(LED_PIN, r, g, b);
#else
  neopixelWrite(LED_PIN, r, g, b);
#endif
}

void sendTelemetry(unsigned long echoTs) {
  char msg[48];
  int n = snprintf(msg, sizeof(msg), "T,%lu,%d", echoTs, (int)WiFi.RSSI());
  udp.beginPacket(TX_IP, TEL_PORT);
  udp.write((const uint8_t*)msg, n);
  udp.endPacket();
}

void readCommands() {
  int size;
  while ((size = udp.parsePacket()) > 0) {
    char buf[64];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = '\0';

    unsigned long seq, ts;
    int thr, ail, ele, arm;
    if (sscanf(buf, "C,%lu,%lu,%d,%d,%d,%d", &seq, &ts, &thr, &ail, &ele, &arm) == 6 &&
        thr >= THR_MIN && thr <= THR_MAX &&
        ail >= SRV_MIN && ail <= SRV_MAX &&
        ele >= SRV_MIN && ele <= SRV_MAX) {
      armed = (arm == 1);
      tgtThr = armed ? thr : THR_MIN;
      tgtAil = ail;
      tgtEle = ele;
      lastValidMs = millis();
      goodPackets++;
      if (seq % 5 == 0) sendTelemetry(ts);   // 10 Hz telemetry, immediate echo
    } else {
      badPackets++;
    }
  }
}

int stepToward(int cur, int tgt, int up, int down) {
  if (tgt > cur) return min(tgt, cur + up);
  if (tgt < cur) return max(tgt, cur - down);
  return cur;
}

void setup() {
  Serial.begin(115200);
  setLed(20, 0, 0);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  esc.setPeriodHertz(50);
  servoAil.setPeriodHertz(50);
  servoEle.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);
  servoAil.attach(AIL_PIN, 1000, 2000);
  servoEle.attach(ELE_PIN, 1000, 2000);
  esc.writeMicroseconds(THR_MIN);      // minimum throttle for ESC arming
  servoAil.writeMicroseconds(SRV_MID);
  servoEle.writeMicroseconds(SRV_MID);

  WiFi.mode(WIFI_STA);
  WiFi.config(LOCAL_IP, GATEWAY, SUBNET);
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to TX");
  while (WiFi.status() != WL_CONNECTED) {
    esc.writeMicroseconds(THR_MIN);
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nConnected, IP %s RSSI %d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  udp.begin(CMD_PORT);
  lastValidMs = millis();
}

void loop() {
  readCommands();

  failsafe = (millis() - lastValidMs > FAILSAFE_MS);

  if (millis() - lastStep >= 10) {
    lastStep = millis();
    if (failsafe || !armed) {
      curThr = THR_MIN;                 // immediate cut on failsafe / kill
      tgtThr = THR_MIN;
    } else {
      curThr = stepToward(curThr, tgtThr, 10, THR_DOWN_STEP);
    }
    if (failsafe) { tgtAil = SRV_MID; tgtEle = SRV_MID; }
    curAil = stepToward(curAil, tgtAil, SERVO_STEP, SERVO_STEP);
    curEle = stepToward(curEle, tgtEle, SERVO_STEP, SERVO_STEP);

    esc.writeMicroseconds(curThr);
    servoAil.writeMicroseconds(curAil);
    servoEle.writeMicroseconds(curEle);

    if (failsafe) setLed(20, 0, 0);
    else if (armed) setLed(0, 0, 20);
    else setLed(0, 20, 0);
  }

  if (millis() - lastLog >= 500) {
    lastLog = millis();
    Serial.printf("%s ARM=%d THR=%d AIL=%d ELE=%d RSSI=%d good=%lu bad=%lu\n",
                  failsafe ? "FAILSAFE" : "LINK OK ", armed, curThr, curAil, curEle,
                  (int)WiFi.RSSI(), (unsigned long)goodPackets, (unsigned long)badPackets);
  }
  delay(1);
}
