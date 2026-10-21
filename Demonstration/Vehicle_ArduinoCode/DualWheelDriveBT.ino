// DualWheelDriveBT - closed-loop PI speed control of both wheels of one differential-drive
// robot, commanded by the MATLAB host (FleetLink.m) over an HC-06 on the hardware UART.
// Commands, one per line: S<wL>:<wR> wheel speeds [rad/s], h heartbeat, x stop,
// T<ms> telemetry period, KP<v> / KI<v> set gain, V<id> set id, ? identity.

// ================= CONFIGURATION =================
// Encoder input pins, left and right (INT0, INT1)
const uint8_t ENC_L_PIN = 2;
const uint8_t ENC_R_PIN = 3;
// Motor PWM pins, left and right (Timer1 OC1A, OC1B)
const uint8_t PWM_L_PIN = 9;
const uint8_t PWM_R_PIN = 10;

// Internal pull-ups on the encoder inputs (true for open-collector sensors)
const bool     USE_INTERNAL_PULLUP = false;
// Encoder settling delay at start-up [ms]
const uint16_t ENC_SETTLE_MS       = 100;

// Supply voltage, measured with the pack loaded [V]
const float SUPPLY_V      = 8.0;
// L298N driver voltage drop at full duty [V]
const float DRIVER_DROP_V = 1.4;
// Motor rated voltage, the hard output ceiling [V]
const float V_MOTOR_MAX   = 6.6;

// Encoder slots per encoder revolution
const uint16_t SLOTS_PER_REV    = 20;
// Count both edges of each slot (true) or rising edges only (false)
const bool     COUNT_BOTH_EDGES = true;
// Encoder revolutions per wheel revolution
const float    GEAR_RATIO       = 1.0;

// Control sample period [ms]
const uint16_t TICK_MS      = 20;
// Speed estimator window [ticks]
const uint8_t  WINDOW_TICKS = 3;
// Glitch filter: reject edges closer than this fraction of the shortest slot pulse
const float    MIN_PULSE_FRACTION = 0.25f;
// Smallest slot duty cycle assumed by the glitch filter
const float    SLOT_DUTY_MIN      = 0.10f;
// No encoder edge for this long means the wheel is stopped [ms]
const uint32_t STALL_TIMEOUT_MS   = 500;
// Wheel speed setpoint limit [rad/s]
const float    W_COMMAND_MAX      = 60.0f;

// Wheel speed PI gains, used until the host pushes KP/KI [V*s/rad], [V/rad]
const float KP_INIT = 0.18f;
const float KI_INIT = 2.5f;

// Stall trip: setpoint above and measured speed below this, at full output [rad/s]
const float    STALL_W_MIN   = 1.0f;
// Stall trip: time stalled at full output before the wheel is stopped [ms]
const uint16_t STALL_TRIP_MS = 1200;

// Link watchdog: stop both wheels if no valid command arrives for this long [ms]
const uint16_t LINK_TIMEOUT_MS = 600;

// Telemetry period at start-up [ms] (0 = off)
const uint16_t TELEM_MS_INIT = 100;

// Serial baud rate (the HC-06 is set to match with AT+BAUD8)
const uint32_t BAUD = 115200;
// Run an unterminated command after this long with no new characters [ms]
const uint16_t CMD_IDLE_MS = 250;
// Telemetry column separator
#define DELIMITER ","

// Vehicle id at start-up (the host sets it at connect with V<id>)
const uint8_t VEHICLE_ID_INIT = 3;


// ================= DERIVED =================
// Encoder pulses per wheel revolution, and wheel angle per pulse [rad]
const float PULSES_PER_WHEEL_REV =
    (float)SLOTS_PER_REV * (COUNT_BOTH_EDGES ? 2.0f : 1.0f) * GEAR_RATIO;
const float RAD_PER_PULSE = TWO_PI / PULSES_PER_WHEEL_REV;

// Control period and stopped-wheel timeout [us]
const uint32_t TICK_US          = (uint32_t)TICK_MS * 1000UL;
const uint32_t STALL_TIMEOUT_US = STALL_TIMEOUT_MS * 1000UL;

// Slot period at W_COMMAND_MAX, and the shortest edge spacing accepted [us]
const float    SLOT_PERIOD_MIN_US =
    1.0e6f / (W_COMMAND_MAX / TWO_PI * PULSES_PER_WHEEL_REV);
const uint32_t MIN_PULSE_US = (uint32_t)(SLOT_PERIOD_MIN_US
                                         * SLOT_DUTY_MIN * MIN_PULSE_FRACTION);

// Bridge output at full duty, and the controller output limit [V]
const float V_BRIDGE_MAX  = SUPPLY_V - DRIVER_DROP_V;
const float V_COMMAND_MAX = (V_BRIDGE_MAX < V_MOTOR_MAX) ? V_BRIDGE_MAX
                                                         : V_MOTOR_MAX;

// Limits on the measured control interval [s]
const float DT_MIN_S = (float)TICK_MS * 0.0005f;
const float DT_MAX_S = (float)TICK_MS * 0.0040f;

// Wheel indices
const uint8_t L = 0;
const uint8_t R = 1;
const uint8_t NWHEEL = 2;


// ================= STATE =================
// Edge count, last edge time [us], edge-seen flag and last edge interval [us], written by the ISRs
volatile uint32_t g_count     [NWHEEL] = {0, 0};
volatile uint32_t g_lastEdgeUs[NWHEEL] = {0, 0};
volatile bool     g_haveEdge  [NWHEEL] = {false, false};
volatile uint32_t g_lastIntervalUs[NWHEEL] = {0, 0};

// Edge count and last edge time [us] at each of the last WINDOW_TICKS + 1 ticks
uint32_t g_ringCount [NWHEEL][WINDOW_TICKS + 1];
uint32_t g_ringEdgeUs[NWHEEL][WINDOW_TICKS + 1];
uint8_t  g_ringHead = 0;
uint8_t  g_ringFill = 0;

// Motor voltage applied, after duty quantisation [V]
float g_cmdV[NWHEEL] = {0.0f, 0.0f};

// PI gains [V*s/rad], [V/rad]
struct Gains {
  float kp, ki;
};
Gains g_k = { KP_INIT, KI_INIT };

// PI state per wheel; sat is +1 when pinned at V_COMMAND_MAX, -1 when asking below 0 V
struct Pid {
  float  iTerm, ePrev;
  bool   primed;
  int8_t sat;
};
Pid g_pid[NWHEEL];

// Wheel speed setpoints [rad/s], stall timers [ms] and the stall-trip flag
float    g_setW   [NWHEEL] = {0.0f, 0.0f};
uint16_t g_stallMs[NWHEEL] = {0, 0};
bool     g_stallTripped = false;

// Command line buffer
char     g_cmd[24];
uint8_t  g_cmdLen = 0;
// Time of the last valid command [ms]; the link watchdog is armed by the first one
uint32_t g_lastCmdMs = 0;
bool     g_linkArmed = false;
bool     g_linkLost  = false;
// Telemetry period and next send time [ms]
uint16_t g_telemMs   = TELEM_MS_INIT;
uint32_t g_telemNext = 0;
// Vehicle id, reported in telemetry column 1
uint8_t  g_vehId     = VEHICLE_ID_INIT;


// Function prototypes (the Arduino IDE generates these, plain avr-g++ does not)
void isrLeft();
void isrRight();
void setupMotorPwm();
float clampW(float w);
void setMotorVolts(uint8_t w, float volts);
void pidReset(uint8_t w);
void pidResetAll();
float pidUpdate(uint8_t w, float sp, float y, float dt);
void setSetpoint(uint8_t w, float rads);
void setBothSetpoints(float wL, float wR);
void allStop();
void stallWatch(const float *omega, float dt);
void linkWatch();
void printIdentity();
void sendTelemetry(const float *omega);
void handleSerial();
static bool dispatch(const char *s);
void execCommand(const char *s);


// ================= ENCODER INTERRUPTS =================
// Record one encoder edge on wheel w, ignoring edges closer than MIN_PULSE_US
static inline void edgeSeen(uint8_t w) __attribute__((always_inline));
static inline void edgeSeen(uint8_t w)
{
  const uint32_t now = micros();

  if (g_haveEdge[w] && (now - g_lastEdgeUs[w]) < MIN_PULSE_US) return;

  g_lastIntervalUs[w] = g_haveEdge[w] ? (now - g_lastEdgeUs[w]) : 0;
  g_lastEdgeUs[w]     = now;
  g_haveEdge[w]       = true;
  g_count[w]++;
}

void isrLeft()  { edgeSeen(L); }
void isrRight() { edgeSeen(R); }


// ================= MOTOR OUTPUT =================
// Timer1 8-bit phase-correct PWM at 31372 Hz on D9/D10, starting at zero duty
void setupMotorPwm()
{
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);   // 8-bit phase correct
  TCCR1B = _BV(CS10);                                // no prescale
  OCR1A  = 0;
  OCR1B  = 0;
}


// Clamp a wheel speed setpoint to 0..W_COMMAND_MAX [rad/s] (forward only)
float clampW(float w)
{
  if (w < 0.0f) return 0.0f;
  if (w > W_COMMAND_MAX) return W_COMMAND_MAX;
  return w;
}


// Apply a motor voltage clamped to 0..V_COMMAND_MAX [V]; duty is scaled against V_BRIDGE_MAX
void setMotorVolts(uint8_t w, float volts)
{
  if (volts < 0.0f) volts = 0.0f;
  if (volts > V_COMMAND_MAX) volts = V_COMMAND_MAX;

  const uint8_t duty = (uint8_t)(volts / V_BRIDGE_MAX * 255.0f + 0.5f);

  g_cmdV[w] = (float)duty / 255.0f * V_BRIDGE_MAX;
  analogWrite(w == L ? PWM_L_PIN : PWM_R_PIN, duty);
}


// ================= PI CONTROLLER =================
// Clear wheel w's controller state and stall timer
void pidReset(uint8_t w)
{
  Pid &s = g_pid[w];
  s.iTerm  = 0.0f;
  s.ePrev  = 0.0f;
  s.primed = false;
  s.sat    = 0;
  g_stallMs[w] = 0;
}


// Reset both wheels' controllers
void pidResetAll()
{
  for (uint8_t w = 0; w < NWHEEL; w++) pidReset(w);
}


// PI update for wheel w: setpoint sp and speed y [rad/s], interval dt [s] -> motor voltage [V]
float pidUpdate(uint8_t w, float sp, float y, float dt)
{
  Pid &s = g_pid[w];

  // Zero setpoint: hold the controller reset and coast (the bridge cannot brake)
  if (sp <= 0.0f) {
    pidReset(w);
    s.primed = true;
    return 0.0f;
  }

  const float e = sp - y;

  if (!s.primed) {
    s.ePrev  = e;
    s.primed = true;
  }

  const float pTerm = g_k.kp * e;

  // Trapezoidal integral, clamped to +-V_COMMAND_MAX, held while it would deepen output saturation
  const float inc = g_k.ki * 0.5f * (e + s.ePrev) * dt;
  float iCand = s.iTerm + inc;
  if (iCand >  V_COMMAND_MAX) iCand =  V_COMMAND_MAX;
  if (iCand < -V_COMMAND_MAX) iCand = -V_COMMAND_MAX;

  const float uCand    = pTerm + iCand;
  const bool  pushUp   = (uCand > V_COMMAND_MAX) && (inc > 0.0f);
  const bool  pushDown = (uCand < 0.0f)          && (inc < 0.0f);
  if (!pushUp && !pushDown) s.iTerm = iCand;
  s.ePrev = e;

  // Output, saturation flag for telemetry, then clamp to 0..V_COMMAND_MAX
  float u = pTerm + s.iTerm;
  s.sat = (u > V_COMMAND_MAX) ? 1 : ((u < 0.0f) ? -1 : 0);
  if (u < 0.0f)          u = 0.0f;
  if (u > V_COMMAND_MAX) u = V_COMMAND_MAX;
  return u;
}


// Set wheel w's speed setpoint [rad/s] and clear its stall state (the controller is not reset)
void setSetpoint(uint8_t w, float rads)
{
  g_setW[w] = clampW(rads);
  g_stallMs[w] = 0;
  g_stallTripped = false;
}


// Set both wheel speed setpoints together so they change on the same tick [rad/s]
void setBothSetpoints(float wL, float wR)
{
  setSetpoint(L, wL);
  setSetpoint(R, wR);
}


// Zero both setpoints, reset both controllers and cut both motors
void allStop()
{
  for (uint8_t w = 0; w < NWHEEL; w++) {
    g_setW[w] = 0.0f;
    pidReset(w);
    setMotorVolts(w, 0.0f);
  }
}


// ================= WATCHDOGS =================
// Stall trip: stop a wheel held at full output below STALL_W_MIN for STALL_TRIP_MS
void stallWatch(const float *omega, float dt)
{
  const uint16_t dtMs = (uint16_t)(dt * 1000.0f + 0.5f);

  for (uint8_t w = 0; w < NWHEEL; w++) {
    const bool bad = (g_setW[w] > STALL_W_MIN)
                  && (omega[w]  < STALL_W_MIN)
                  && (g_cmdV[w] >= V_COMMAND_MAX - 0.05f);

    if (!bad) { g_stallMs[w] = 0; continue; }

    g_stallMs[w] = (uint16_t)((g_stallMs[w] + dtMs > 60000u)
                                ? 60000u : g_stallMs[w] + dtMs);
    if (g_stallMs[w] < STALL_TRIP_MS) continue;

    g_setW[w] = 0.0f;
    pidReset(w);
    setMotorVolts(w, 0.0f);
    g_stallTripped = true;
  }
}


// Link watchdog: stop both wheels, once per outage, after LINK_TIMEOUT_MS with no valid command
void linkWatch()
{
  if (!g_linkArmed) return;

  if ((millis() - g_lastCmdMs) < LINK_TIMEOUT_MS) return;

  if (!g_linkLost) {
    g_linkLost = true;
    allStop();
    Serial.println(F("# link lost"));
  }
}


// ================= REPORTING =================
// Print the identity line: "# BT v<id> wmax <rad/s> vmax <V> tick <ms> lto <ms>"
void printIdentity()
{
  Serial.print(F("# BT v"));
  Serial.print(g_vehId);
  Serial.print(F(" wmax "));
  Serial.print(W_COMMAND_MAX, 1);
  Serial.print(F(" vmax "));
  Serial.print(V_COMMAND_MAX, 2);
  Serial.print(F(" tick "));
  Serial.print(TICK_MS);
  Serial.print(F(" lto "));
  Serial.println(LINK_TIMEOUT_MS);
}


// Send one telemetry row: id,tMs,spL,wL,uL,spR,wR,uR,flags (tMs [ms], sp/w [rad/s], u [V])
// flags: 1 L at V_COMMAND_MAX, 2 L asking below 0 V, 4/8 same for R, 16 link lost, 32 stall trip
void sendTelemetry(const float *omega)
{
  uint8_t flags = 0;
  if (g_pid[L].sat > 0) flags |= 1;
  if (g_pid[L].sat < 0) flags |= 2;
  if (g_pid[R].sat > 0) flags |= 4;
  if (g_pid[R].sat < 0) flags |= 8;
  if (g_linkLost)       flags |= 16;
  if (g_stallTripped)   flags |= 32;

  Serial.print(g_vehId);            Serial.print(F(DELIMITER));
  Serial.print(millis());           Serial.print(F(DELIMITER));
  Serial.print(g_setW[L], 2);       Serial.print(F(DELIMITER));
  Serial.print(omega[L], 3);        Serial.print(F(DELIMITER));
  Serial.print(g_cmdV[L], 3);       Serial.print(F(DELIMITER));
  Serial.print(g_setW[R], 2);       Serial.print(F(DELIMITER));
  Serial.print(omega[R], 3);        Serial.print(F(DELIMITER));
  Serial.print(g_cmdV[R], 3);       Serial.print(F(DELIMITER));
  Serial.println(flags);
}


// ================= COMMANDS =================
// Buffer serial input; run a line on CR/LF or after CMD_IDLE_MS idle; 'x' stops at once
void handleSerial()
{
  static uint32_t lastCharMs = 0;

  if (g_cmdLen > 0 && (millis() - lastCharMs) >= CMD_IDLE_MS) {
    g_cmd[g_cmdLen] = '\0';
    execCommand(g_cmd);
    g_cmdLen = 0;
  }

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    lastCharMs = millis();

    // Emergency stop, honoured anywhere in the stream; discards any partial command
    if (c == 'x' || c == 'X') {
      g_cmdLen = 0;
      execCommand("x");
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (g_cmdLen > 0) { g_cmd[g_cmdLen] = '\0'; execCommand(g_cmd); }
      g_cmdLen = 0;
      continue;
    }

    if (c == ' ' || c == '\t') continue;
    if (c < 32 || c > 126)     continue;      // drop line noise

    if (g_cmdLen >= sizeof(g_cmd) - 1) {
      g_cmdLen = 0;                            // discard, do not truncate
      Serial.println(F("# ? long"));
      continue;
    }
    g_cmd[g_cmdLen++] = c;
  }
}


// Run one command; returns false if it is not understood
static bool dispatch(const char *s)
{
  const char c = s[0];
  const char *a = s + 1;                       // the argument, if any

  switch (c) {

    case 'x':                                  // stop
      allStop();
      Serial.println(F("# stop"));
      return true;

    case 'h':                                  // heartbeat
      return true;

    case 'S': {                                // step: S<wL>:<wR> [rad/s]
      char *end;
      const float wL = strtod(a, &end);
      if (end == a) break;
      while (*end == ':') end++;
      if (*end == '\0') break;
      const char *b = end;
      const float wR = strtod(b, &end);
      if (end == b) break;
      setBothSetpoints(wL, wR);
      return true;
    }

    case 'K': {                                // KP<v> or KI<v>
      if (a[0] == '\0') break;
      const float v = atof(a + 1);
      switch (a[0]) {
        case 'P': g_k.kp = v; return true;
        case 'I': g_k.ki = v; return true;
      }
      break;
    }

    case 'T': g_telemMs = (uint16_t)atol(a); return true;

    case 'V': {                                // vehicle id, 1..9
      const uint8_t id = (uint8_t)atoi(a);
      if (id >= 1 && id <= 9) { g_vehId = id; return true; }
      break;
    }

    case '?': if (*a == '\0') { printIdentity(); return true; } break;
  }

  return false;
}


// Run a command: reply "# ?<cmd>" if not understood, otherwise refresh the link watchdog
void execCommand(const char *s)
{
  if (!dispatch(s)) {
    Serial.print(F("# ?"));
    Serial.println(s);
    return;
  }

  g_lastCmdMs = millis();
  g_linkArmed = true;
  g_linkLost  = false;
}


// ================= SETUP =================
void setup()
{
  // Zero duty before the PWM pins become outputs
  setupMotorPwm();
  pinMode(PWM_L_PIN, OUTPUT);  digitalWrite(PWM_L_PIN, LOW);
  pinMode(PWM_R_PIN, OUTPUT);  digitalWrite(PWM_R_PIN, LOW);
  setMotorVolts(L, 0.0f);
  setMotorVolts(R, 0.0f);

  Serial.begin(BAUD);

  pinMode(ENC_L_PIN, USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(ENC_R_PIN, USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  delay(ENC_SETTLE_MS);

  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isrLeft,
                  COUNT_BOTH_EDGES ? CHANGE : RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isrRight,
                  COUNT_BOTH_EDGES ? CHANGE : RISING);

  pidResetAll();
  printIdentity();
}


// ================= MAIN LOOP =================
// Every pass: serial and link watchdog. Every TICK_MS: speed estimate, PI, stall check, telemetry
void loop()
{
  static uint32_t nextTickUs = 0;
  static uint32_t lastCtlUs  = 0;
  static bool     started    = false;
  static bool     ctlPrimed  = false;

  handleSerial();
  linkWatch();

  // Tick gate (signed difference survives micros() rollover; accumulating avoids drift)
  if (!started) { nextTickUs = micros(); started = true; }
  if ((int32_t)(micros() - nextTickUs) < 0) return;
  nextTickUs += TICK_US;

  // Snapshot both encoders at the same instant
  uint32_t nowUs, cnt[NWHEEL], edge[NWHEEL], ivl[NWHEEL];
  bool     have[NWHEEL];
  noInterrupts();
  nowUs   = micros();
  cnt[L]  = g_count[L];          cnt[R]  = g_count[R];
  edge[L] = g_lastEdgeUs[L];     edge[R] = g_lastEdgeUs[R];
  ivl[L]  = g_lastIntervalUs[L]; ivl[R]  = g_lastIntervalUs[R];
  have[L] = g_haveEdge[L];       have[R] = g_haveEdge[R];
  interrupts();

  // Store this tick's counts and edge times in the estimator history
  g_ringHead = (uint8_t)((g_ringHead + 1) % (WINDOW_TICKS + 1));
  g_ringCount [L][g_ringHead] = cnt[L];
  g_ringCount [R][g_ringHead] = cnt[R];
  g_ringEdgeUs[L][g_ringHead] = edge[L];
  g_ringEdgeUs[R][g_ringHead] = edge[R];
  if (g_ringFill <= WINDOW_TICKS) g_ringFill++;

  // Wheel speeds [rad/s]
  float omega[NWHEEL] = {0.0f, 0.0f};

  for (uint8_t w = 0; w < NWHEEL; w++) {

    // M/T estimate: pulses over edge-to-edge time, using the longest span with new pulses
    bool haveMT = false;
    for (uint8_t back = (uint8_t)(g_ringFill >= 2 ? g_ringFill - 1 : 0);
         back >= 1; back--) {

      const uint8_t idx = (uint8_t)((g_ringHead + (WINDOW_TICKS + 1) - back)
                                    % (WINDOW_TICKS + 1));

      if (g_ringCount[w][idx] < 1)        continue;   // no edge existed yet
      if (cnt[w] <= g_ringCount[w][idx])  continue;   // no new pulse since

      const uint32_t dtEdgeUs = edge[w] - g_ringEdgeUs[w][idx];
      if (dtEdgeUs == 0)                  continue;

      omega[w] = (float)(cnt[w] - g_ringCount[w][idx])
                 * RAD_PER_PULSE * 1.0e6f / (float)dtEdgeUs;
      haveMT = true;
      break;
    }

    // T fallback: one pulse over the last edge interval, decaying as 1/t since the last edge
    if (!haveMT && have[w] && ivl[w] > 0) {
      const uint32_t sinceUs = nowUs - edge[w];
      if (sinceUs < STALL_TIMEOUT_US) {
        const uint32_t effUs = (sinceUs > ivl[w]) ? sinceUs : ivl[w];
        omega[w] = RAD_PER_PULSE * 1.0e6f / (float)effUs;
      }
    }

    // Staleness ceiling: no edge for sinceUs bounds the speed to one pulse per sinceUs
    if (have[w]) {
      const uint32_t sinceUs = nowUs - edge[w];
      if (sinceUs >= STALL_TIMEOUT_US) {
        omega[w] = 0.0f;
      } else if (sinceUs > 0) {
        const float bound = RAD_PER_PULSE * 1.0e6f / (float)sinceUs;
        if (omega[w] > bound) omega[w] = bound;
      }
    }
  }

  // Measured control interval, clamped to DT_MIN_S..DT_MAX_S [s]
  float dt = (float)TICK_MS / 1000.0f;
  if (ctlPrimed) dt = (float)(nowUs - lastCtlUs) * 1.0e-6f;
  ctlPrimed = true;
  lastCtlUs = nowUs;
  if (dt < DT_MIN_S) dt = DT_MIN_S;
  if (dt > DT_MAX_S) dt = DT_MAX_S;

  for (uint8_t w = 0; w < NWHEEL; w++) {
    setMotorVolts(w, pidUpdate(w, g_setW[w], omega[w], dt));
  }

  // After the outputs are written, so g_cmdV holds this tick's value
  stallWatch(omega, dt);

  if (g_telemMs > 0 && (int32_t)(millis() - g_telemNext) >= 0) {
    g_telemNext = millis() + g_telemMs;
    sendTelemetry(omega);
  }
}
