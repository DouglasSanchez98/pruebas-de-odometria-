#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>

/* =========================================================
 *                ENUM PARA REVERSA (solo una vez)
 * ========================================================= */
enum RevState { REV_NONE = 0, REV_1, REV_2 };

/* =========================================================
 *                         PINES
 * ========================================================= */
#define THR_A_PIN      4    // PWM motor A
#define THR_B_PIN      16   // PWM motor B 
#define REVERSA1_PIN   18   // Dirección/Reversa 1
#define REVERSA2_PIN   19   // Dirección/Reversa 2
#define DAC1_PIN       25   // DAC analógico (0–3.3 V)

// Sensores Hall motor A
#define H1_PIN         32
#define H2_PIN         33
#define H3_PIN         27

/* =========================================================
 *                PWM / RAMPAS / TIEMPOS
 * ========================================================= */
const uint8_t  PWM_MIN_8B      = 0;
const uint8_t  PWM_MAX_8B      = 255;

const uint32_t LEDC_FREQ_HZ    = 20000;  // 20 kHz
const uint8_t  LEDC_RES_BITS   = 8;      // 0..255

// Rampa solo para FRENAR (no para acelerar)
const uint16_t RAMP_MS_PER_STEP = 4;     // cuanto baja por paso
const uint32_t CTRL_DT_MS       = 5;     // período de la task de control

/* =========================================================
 *        RTOS: handles de tareas y cola de comandos
 * ========================================================= */
TaskHandle_t   hTaskSerial = nullptr;
TaskHandle_t   hTaskCtrl   = nullptr;
QueueHandle_t  qCmd        = nullptr;

/* =========================================================
 *                       COMANDOS
 * ========================================================= */
enum CmdType : uint8_t {
  CMD_SET_A,      // PWM motor A
  CMD_SET_B,      // PWM motor B
  CMD_SET_D,      // PWM ambos
  CMD_STOP_BOTH,  // parar ambos con rampa
  CMD_REV_0,      // sin reversa
  CMD_REV_1,      // reversa 1
  CMD_REV_2,      // reversa 2
  CMD_SET_DAC     // salida DAC en pin 25
};

struct Command {
  CmdType  type;
  uint16_t value;   // 0..255
};

/* =========================================================
 *        ESTADO DEL CONTROL (PWM + REVERSA)
 * ========================================================= */

static float   currA_8b = 0.0f,   currB_8b = 0.0f;  // valor aplicado ahora
static float   tgtA_8b  = 0.0f,   tgtB_8b  = 0.0f;  // setpoint 0..255

static RevState desiredRev = REV_NONE;
static RevState activeRev  = REV_NONE;

static const float ramp_steps_per_ms = 1.0f / (float)RAMP_MS_PER_STEP;
static const float EPS_ZERO          = 1.0f;

/* =========================================================
 *         ODOMETRÍA / RPM (motor A con 3 Halls)
 * ========================================================= */

// Perímetro de la rueda [m]  (74.35 cm medidos)
const double CIRC_M = 0.7435;

// CPR = “counts per revolution” = cambios de estado válidos por vuelta
// 30 imanes * 3 halls = 90
const int    CPR          = 90;

// Filtros/ventana
const unsigned long MIN_TICK_US = 150;   // mínimo tiempo entre flancos
const unsigned long WINDOW_MS   = 250;   // ventana para cálculo (0.25 s)

// Variables compartidas con la ISR
volatile uint8_t       hall_last_state = 0xFF;
volatile unsigned long hall_tick_count = 0;

// Leer estado de los 3 Halls como 3 bits
static inline uint8_t hall_read_state() {
  uint8_t b1 = digitalRead(H1_PIN);
  uint8_t b2 = digitalRead(H2_PIN);
  uint8_t b3 = digitalRead(H3_PIN);
  return (uint8_t)((b1 << 2) | (b2 << 1) | b3);
}

// ISR: cuenta ticks válidos (motor A)
void IRAM_ATTR hall_isr() {
  static unsigned long last_tick_us = 0;

  uint8_t s = hall_read_state();
  if (s == 0b000 || s == 0b111) return;   // estados inválidos

  unsigned long now = micros();

  if (hall_last_state == 0xFF) {
    hall_last_state = s;
    last_tick_us    = now;
    return;
  }

  if (s != hall_last_state) {
    unsigned long dt = now - last_tick_us;
    if (dt >= MIN_TICK_US) {
      hall_tick_count++;       // un tick más en la ventana
      last_tick_us    = now;
      hall_last_state = s;
    }
  }
}

/* =========================================================
 *                      UTILIDADES
 * ========================================================= */

static inline bool bothAtZero() {
  return (fabsf(currA_8b) <= EPS_ZERO) && (fabsf(currB_8b) <= EPS_ZERO)
      && (fabsf(tgtA_8b)  <= EPS_ZERO) && (fabsf(tgtB_8b)  <= EPS_ZERO);
}

static inline void setReversePins(RevState r) {
  switch (r) {
    case REV_NONE:
      digitalWrite(REVERSA1_PIN, LOW);
      digitalWrite(REVERSA2_PIN, LOW);
      break;
    case REV_1:
      digitalWrite(REVERSA1_PIN, HIGH);
      digitalWrite(REVERSA2_PIN, LOW);
      break;
    case REV_2:
      digitalWrite(REVERSA1_PIN, LOW);
      digitalWrite(REVERSA2_PIN, HIGH);
      break;
  }
}

// Aplicar PWM a los dos motores (0..255)
// Con la API nueva: ledcAttach(pin, freq, res_bits); ledcWrite(pin, duty);
static inline void applyPWM_LEDC() {
  uint32_t dA = (currA_8b < 0)   ? 0 :
                (currA_8b > 255) ? 255 : (uint32_t)currA_8b;

  uint32_t dB = (currB_8b < 0)   ? 0 :
                (currB_8b > 255) ? 255 : (uint32_t)currB_8b;

  ledcWrite(THR_A_PIN, dA);
  ledcWrite(THR_B_PIN, dB);
}

/* =========================================================
 *                TAREA: CONTROL (core 1)
 * ========================================================= */

void taskControl(void *pv) {
  TickType_t t0 = xTaskGetTickCount();

  // Para cálculo de RPM por ventanas
  unsigned long last_ticks   = 0;
  TickType_t    last_winTick = xTaskGetTickCount();
  const TickType_t WIN_TICKS = pdMS_TO_TICKS(WINDOW_MS);

  for (;;) {
    /* ---------- Procesar comandos de la cola ---------- */
    Command cmd;
    while (xQueueReceive(qCmd, &cmd, 0) == pdTRUE) {
      switch (cmd.type) {
        case CMD_SET_A:
          tgtA_8b = constrain((int)cmd.value, PWM_MIN_8B, PWM_MAX_8B);
          break;

        case CMD_SET_B:
          tgtB_8b = constrain((int)cmd.value, PWM_MIN_8B, PWM_MAX_8B);
          break;

        case CMD_SET_D:   // ambos al mismo valor
          tgtA_8b = constrain((int)cmd.value, PWM_MIN_8B, PWM_MAX_8B);
          tgtB_8b = constrain((int)cmd.value, PWM_MIN_8B, PWM_MAX_8B);
          break;

        case CMD_STOP_BOTH:
          tgtA_8b = 0.0f;
          tgtB_8b = 0.0f;
          break;

        case CMD_REV_0:
          desiredRev = REV_NONE;
          setReversePins(REV_NONE);
          activeRev = REV_NONE;
          break;

        case CMD_REV_1:
          desiredRev = REV_1;
          tgtA_8b = 0.0f;
          tgtB_8b = 0.0f;
          break;

        case CMD_REV_2:
          desiredRev = REV_2;
          tgtA_8b = 0.0f;
          tgtB_8b = 0.0f;
          break;

        case CMD_SET_DAC:
          dacWrite(DAC1_PIN, cmd.value);
          Serial.printf("DAC Pin %d = %d (%.2f V aprox)\n",
                        DAC1_PIN, cmd.value, (3.3 * cmd.value / 255.0));
          break;
      }
    }

    /* ---------- Rampa solo al frenar ---------- */
    const float dt_ms = (float)CTRL_DT_MS;
    const float step  = ramp_steps_per_ms * dt_ms;

    auto rampTo = [&](float curr, float tgt) -> float {
      if (tgt >= curr) {
        return tgt;  // subida directa (sin rampa)
      } else {
        if (fabsf(tgt - curr) <= step) return tgt;
        return curr - step;
      }
    };

    currA_8b = rampTo(currA_8b, tgtA_8b);
    currB_8b = rampTo(currB_8b, tgtB_8b);

    if (desiredRev != activeRev && bothAtZero()) {
      setReversePins(desiredRev);
      activeRev = desiredRev;
    }

    applyPWM_LEDC();

    /* ---------- Cada WINDOW_MS ms: calcular RPM y v[m/s] ---------- */
    TickType_t nowTick = xTaskGetTickCount();
    if ((nowTick - last_winTick) >= WIN_TICKS) {
      last_winTick = nowTick;

      unsigned long ticks_now;
      noInterrupts();
      ticks_now = hall_tick_count;
      interrupts();

      unsigned long delta_ticks = ticks_now - last_ticks;
      last_ticks = ticks_now;

      double rpm  = 0.0;
      double v_ms = 0.0;

      if (delta_ticks > 0) {
        double T = (double)WINDOW_MS / 1000.0; // segundos de ventana
        rpm  = ((double)delta_ticks * 60.0) / ((double)CPR * T);
        v_ms = (CIRC_M * rpm) / 60.0;
      }

      Serial.printf("A=%.0f  B=%.0f  R1=%d  R2=%d  |  ticks=%lu  RPM=%.2f  v=%.3f m/s\n",
                    currA_8b, currB_8b,
                    digitalRead(REVERSA1_PIN),
                    digitalRead(REVERSA2_PIN),
                    (unsigned long)delta_ticks,
                    rpm,
                    v_ms);
    }

    vTaskDelayUntil(&t0, pdMS_TO_TICKS(CTRL_DT_MS));
  }
}

/* =========================================================
 *          TAREA: SERIAL / PARSER (core 0)
 * ========================================================= */

void taskSerial(void *pv) {
  String linea;
  linea.reserve(32);

  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\n' || c == '\r') {
        if (linea.length() == 0) continue;
        linea.trim();

        Command cmd{};
        bool recognized = true;

        if (linea == "0") {
          cmd.type = CMD_STOP_BOTH;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea.equalsIgnoreCase("R1")) {
          cmd.type = CMD_REV_1;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea.equalsIgnoreCase("R2")) {
          cmd.type = CMD_REV_2;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea.equalsIgnoreCase("R0")) {
          cmd.type = CMD_REV_0;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea[0] == 'A' || linea[0] == 'a') {
          int val = linea.substring(1).toInt();
          val = constrain(val, PWM_MIN_8B, PWM_MAX_8B);
          cmd.type  = CMD_SET_A;
          cmd.value = val;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea[0] == 'B' || linea[0] == 'b') {
          int val = linea.substring(1).toInt();
          val = constrain(val, PWM_MIN_8B, PWM_MAX_8B);
          cmd.type  = CMD_SET_B;
          cmd.value = val;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea[0] == 'D' || linea[0] == 'd') {   // ambos motores mismo PWM
          int val = linea.substring(1).toInt();
          val = constrain(val, PWM_MIN_8B, PWM_MAX_8B);
          cmd.type  = CMD_SET_D;
          cmd.value = val;
          xQueueSend(qCmd, &cmd, 0);

        } else if (linea[0] == 'P' || linea[0] == 'p') {   // DAC
          int val = linea.substring(1).toInt();
          val = constrain(val, 0, 255);
          cmd.type  = CMD_SET_DAC;
          cmd.value = val;
          xQueueSend(qCmd, &cmd, 0);

        } else {
          recognized = false;
          Serial.println(F("Comando invalido"));
        }

        linea = "";

      } else {
        linea += c;
        if (linea.length() > 24) linea = "";  
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); 
  }
}

/* =========================================================
 *                    SETUP / LOOP
 * ========================================================= */

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F(
    "Comandos:\n"
    "A### o B### guardan PWM individual (0-255).\n"
    "D### aplica el mismo PWM a ambos motores.\n"
    "P### genera salida DAC en pin 25 (0-255 -> 0–3.3V aprox).\n"
    "R1 activa Reversa1 | R2 activa Reversa2 | R0 apaga ambas.\n"
    "0 detiene ambos con rampa.\n"
    "Ej.: A200  |  B120  |  D200  |  P128\n"
    "RPM y velocidad se calculan sobre el motor A (H1,H2,H3).\n"
  ));

  pinMode(REVERSA1_PIN, OUTPUT);
  pinMode(REVERSA2_PIN, OUTPUT);
  setReversePins(REV_NONE);

  // Pines Hall
  pinMode(H1_PIN, INPUT);
  pinMode(H2_PIN, INPUT);
  pinMode(H3_PIN, INPUT);

  // Interrupciones en Hall (los 3 llaman a la misma ISR)
  attachInterrupt(digitalPinToInterrupt(H1_PIN), hall_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), hall_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H3_PIN), hall_isr, CHANGE);

  // PWM: API nueva (core 3.x)
  ledcAttach(THR_A_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(THR_B_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcWrite(THR_A_PIN, 0);
  ledcWrite(THR_B_PIN, 0);

  // DAC en 0
  dacWrite(DAC1_PIN, 0);

  // Cola de comandos
  qCmd = xQueueCreate(10, sizeof(Command));

  // Tareas en los dos cores
  xTaskCreatePinnedToCore(taskSerial, "SERIAL_PARSER", 4096, nullptr, 2, &hTaskSerial, 0); 
  xTaskCreatePinnedToCore(taskControl, "CONTROL_LOOP", 4096, nullptr, 3, &hTaskCtrl,   1); 
}

void loop() {
  // vacío: todo lo hacen las tareas
}
