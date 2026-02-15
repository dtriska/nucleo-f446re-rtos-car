#include <WiFi.h>
#include <cmath> 
#include "WifiKey.h"

// If wanting to check motor stats simply run
// nc -v 192.168.1.161 5000

const char* ssid = SSID;
const char* password = PASSWORD;

WiFiServer server(5000);

// ===================== Motor / Driver pins =====================
// Motor 1 (L298 channel A)
constexpr int IN1 = 2;   // GP2
constexpr int IN2 = 3;   // GP3
constexpr int ENA = 4;   // GP4 PWM

// Motor 2 (L298 channel B)
constexpr int IN3 = 6;   // GP6
constexpr int IN4 = 7;   // GP7
constexpr int ENB = 8;   // GP8 PWM

// ===================== Encoder pins =====================
// Motor 1 encoder
constexpr int ENC1_A = 10; // GP10
constexpr int ENC1_B = 11; // GP11

// Motor 2 encoder
constexpr int ENC2_A = 12; // GP12
constexpr int ENC2_B = 13; // GP13

volatile int32_t enc1_count = 0;
volatile int32_t enc2_count = 0;

// If forward reads negative RPM, keep -1. If it reads positive, use +1.
constexpr int RPM_SIGN = -1;

// counts per output shaft rev 
constexpr float COUNTS_PER_REV = 2200.0f;

// ===================== Control params (shared base) =====================
volatile float target_rpm = 65.0f;   // base target RPM
volatile float Kp = 1.0f;
volatile float Ki = 4.0f;
volatile float pwm_per_rpm = 3.1f;

constexpr int PWM_MIN = 0;
constexpr int PWM_MAX = 255;

constexpr uint32_t CTRL_DT_MS = 20;
constexpr float    CTRL_DT_S  = CTRL_DT_MS / 1000.0f;
constexpr uint32_t TEL_DT_MS  = 200;

constexpr int PWM_KICK = 40;   // stiction kick

// ===================== Drive modes =====================
enum DriveMode { DM_FWD, DM_BWD, DM_LEFT, DM_RIGHT };
DriveMode mode = DM_FWD;

volatile float turn_scale = 0.25f;

volatile bool enabled = true;

// ===================== Encoder ISRs =====================
void isrEnc1A() 
{
  bool a = digitalRead(ENC1_A);
  bool b = digitalRead(ENC1_B);
  enc1_count += (a == b) ? +1 : -1;
}

void isrEnc2A() 
{
  bool a = digitalRead(ENC2_A);
  bool b = digitalRead(ENC2_B);
  enc2_count += (a == b) ? +1 : -1;
}

// ===================== Motor helpers =====================
static inline void motorStopA() 
{
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
static inline void motorStopB() 
{
  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void applyMotor1(bool forward, int pwm) 
{
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  if (!enabled || pwm == 0) { motorStopA(); return; }

  if (forward) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else         { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }

  analogWrite(ENA, pwm);
}

void applyMotor2(bool forward, int pwm) 
{
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  if (!enabled || pwm == 0) { motorStopB(); return; }

  if (forward) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else         { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }

  analogWrite(ENB, pwm);
}

// ===================== Simple line reader for commands =====================
bool readLine(WiFiClient& c, String& out) 
{
  if (!c.available()) return false;
  out = c.readStringUntil('\n');
  out.trim();
  return out.length() > 0;
}

static char modeChar(DriveMode m) 
{
  switch (m) 
  {
    case DM_FWD:   return 'F';
    case DM_BWD:   return 'B';
    case DM_LEFT:  return 'L';
    case DM_RIGHT: return 'R';
    default:       return '?';
  }
}

void handleCommand(const String& cmd, WiFiClient& client) {
  auto reply = [&](const String& s) { client.println(s); };

  if (cmd == "F") { mode = DM_FWD;   reply("OK MODE=F"); return; }
  if (cmd == "B") { mode = DM_BWD;   reply("OK MODE=B"); return; }
  if (cmd == "L") { mode = DM_LEFT;  reply("OK MODE=L"); return; }
  if (cmd == "R") { mode = DM_RIGHT; reply("OK MODE=R"); return; }

  if (cmd == "?") 
  {
    reply("STATUS:");
    reply("  target_rpm=" + String(target_rpm, 2));
    reply("  Kp=" + String(Kp, 3));
    reply("  Ki=" + String(Ki, 3));
    reply("  FF(pwm_per_rpm)=" + String(pwm_per_rpm, 3));
    reply("  turn_scale=" + String(turn_scale, 3));
    reply(String("  enabled=") + (enabled ? "1" : "0"));
    reply(String("  mode=") + modeChar(mode));
    return;
  }

  float val = 0.0f;

  if (cmd.startsWith("T ")) 
  {
    val = cmd.substring(2).toFloat();
    target_rpm = max(0.0f, val);
    reply("OK T=" + String(target_rpm, 2));
    return;
  }
  if (cmd.startsWith("KP ")) 
  {
    val = cmd.substring(3).toFloat();
    Kp = max(0.0f, val);
    reply("OK KP=" + String(Kp, 3));
    return;
  }
  if (cmd.startsWith("KI ")) 
  {
    val = cmd.substring(3).toFloat();
    Ki = max(0.0f, val);
    reply("OK KI=" + String(Ki, 3));
    return;
  }
  if (cmd.startsWith("FF ")) 
  {
    val = cmd.substring(3).toFloat();
    pwm_per_rpm = max(0.0f, val);
    reply("OK FF=" + String(pwm_per_rpm, 3));
    return;
  }
  if (cmd.startsWith("TS ")) 
  {
    // turn scale: 0.0..1.0
    val = cmd.substring(3).toFloat();
    turn_scale = constrain(val, 0.0f, 1.0f);
    reply("OK TS=" + String(turn_scale, 3));
    return;
  }
  if (cmd.startsWith("EN ")) 
  {
    int en = cmd.substring(3).toInt();
    enabled = (en != 0);
    reply(String("OK EN=") + (enabled ? "1" : "0"));
    return;
  }

  reply("ERR cmds: ?, T <rpm>, KP <v>, KI <v>, FF <v>, TS <0..1>, F, B, L, R, EN 1|0");
}

// ===================== Main =====================
void setup() 
{
  Serial.begin(115200);
  delay(200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrEnc1A, CHANGE);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isrEnc2A, CHANGE);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  server.begin();

  Serial.print("Pico W IP: ");
  Serial.println(WiFi.localIP());

  motorStopA();
  motorStopB();
}

void loop() 
{
  static uint32_t last_ctrl_ms = 0;
  static uint32_t last_tel_ms  = 0;

  static float integ1 = 0.0f;
  static float integ2 = 0.0f;

  static int pwm1_cmd = 0;
  static int pwm2_cmd = 0;

  static WiFiClient client;
  if (!client || !client.connected()) 
  {
    WiFiClient maybe = server.accept();
    if (maybe) {
      client = maybe;
      client.println("Connected. Commands: ?, T <rpm>, KP <v>, KI <v>, FF <v>, TS <0..1>, F, B, L, R, EN 1|0");
    }
  }

  if (client && client.connected()) 
  {
    String line;
    while (readLine(client, line)) handleCommand(line, client);
  }

  const uint32_t now = millis();

  // -------- Determine per-motor direction + per-motor target scaling --------
  bool m1_forward = true;
  bool m2_forward = true;

  float s1 = 1.0f; // target scale for motor1
  float s2 = 1.0f; // target scale for motor2

  switch (mode) 
  {
    case DM_FWD:
      m1_forward = true;  m2_forward = true;
      s1 = 1.0f; s2 = 1.0f;
      break;

    case DM_BWD:
      m1_forward = false; m2_forward = false;
      s1 = 1.0f; s2 = 1.0f;
      break;

    case DM_LEFT:
      // gentle left: both forward, left motor slower
      m1_forward = true;  m2_forward = true;
      s1 = turn_scale;  // left slower
      s2 = 1.0f;
      break;

    case DM_RIGHT:
      // gentle right: both forward, right motor slower
      m1_forward = true;  m2_forward = true;
      s1 = 1.0f;
      s2 = turn_scale;  // right slower
      break;
  }

  // Reset integrators when mode changes (prevents weird carry-over)
  static DriveMode last_mode = DM_FWD;
  if (mode != last_mode) 
  {
    integ1 = 0.0f;
    integ2 = 0.0f;
    last_mode = mode;
  }

  // -------- Control loop --------
  if (now - last_ctrl_ms >= CTRL_DT_MS) 
  {
    last_ctrl_ms = now;

    int32_t c1, c2;
    noInterrupts();
    c1 = enc1_count; enc1_count = 0;
    c2 = enc2_count; enc2_count = 0;
    interrupts();

    float rpm1 = RPM_SIGN * (((float)c1 / COUNTS_PER_REV) / CTRL_DT_S) * 60.0f;
    float rpm2 = RPM_SIGN * (((float)c2 / COUNTS_PER_REV) / CTRL_DT_S) * 60.0f;

    // Per-motor target based on scaling
    float base = fabsf(target_rpm);
    float tgt1 = base * s1;
    float tgt2 = base * s2;

    // Feedforward (per motor)
    float ff1 = pwm_per_rpm * tgt1;
    float ff2 = pwm_per_rpm * tgt2;

    float err1 = tgt1 - fabsf(rpm1);
    float err2 = tgt2 - fabsf(rpm2);

    if (enabled) 
    {
      integ1 = constrain(integ1 + err1 * CTRL_DT_S, -200.0f, 200.0f);
      integ2 = constrain(integ2 + err2 * CTRL_DT_S, -200.0f, 200.0f);
    } 
    else 
    {
      integ1 = 0.0f;
      integ2 = 0.0f;
    }

    float u1 = ff1 + (Kp * err1) + (Ki * integ1);
    float u2 = ff2 + (Kp * err2) + (Ki * integ2);

    pwm1_cmd = constrain((int)lroundf(u1), PWM_MIN, PWM_MAX);
    pwm2_cmd = constrain((int)lroundf(u2), PWM_MIN, PWM_MAX);

    // Stiction kick (only if target > ~1 rpm)
    if (enabled && tgt1 > 1.0f && pwm1_cmd > 0 && pwm1_cmd < PWM_KICK) pwm1_cmd = PWM_KICK;
    if (enabled && tgt2 > 1.0f && pwm2_cmd > 0 && pwm2_cmd < PWM_KICK) pwm2_cmd = PWM_KICK;

    applyMotor1(m1_forward, pwm1_cmd);
    applyMotor2(m2_forward, pwm2_cmd);

    // -------- Telemetry --------
    if (now - last_tel_ms >= TEL_DT_MS) {
      last_tel_ms = now;
      if (client && client.connected()) {
        client.print("rpm1="); client.print(rpm1, 2);
        client.print(" rpm2="); client.print(rpm2, 2);
        client.print(" tgt1="); client.print(tgt1, 2);
        client.print(" tgt2="); client.print(tgt2, 2);
        client.print(" pwm1="); client.print(pwm1_cmd);
        client.print(" pwm2="); client.print(pwm2_cmd);
        client.print(" err1="); client.print(err1, 2);
        client.print(" err2="); client.print(err2, 2);
        client.print(" mode="); client.print(modeChar(mode));
        client.print(" en="); client.println(enabled ? "1" : "0");
      }
    }
  }
}
