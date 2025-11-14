// =========================================
//  CONTROL DE VELOCIDAD (PI) MOTOR A CON HALLS
//  SALIDA POR DAC (pin 25) PARA DRIVER DE HOVERBOARD
// =========================================

#include <Arduino.h>

// ---------- Pines ----------
const int DAC1_PIN      = 25;  // DAC analógico hacia la placa del hoverboard
const int REVERSA1_PIN  = 18;  // Dirección 1
const int REVERSA2_PIN  = 19;  // Dirección 2 (ajusta si hace falta)

const int H1_PIN        = 32;  // Hall 1
const int H2_PIN        = 33;  // Hall 2
const int H3_PIN        = 27;  // Hall 3

// ---------- Parámetros mecánicos ----------
const int   TICKS_PER_REV = 90;       // nº de cambios de patrón Hall por vuelta mecánica
const float CIRC_M        = 0.7435f;  // circunferencia de la rueda [m]

// ---------- Ventana de medida ----------
const unsigned long WINDOW_MS = 250;            // ventana de conteo [ms]
const float         Ts        = WINDOW_MS / 1000.0f;  // periodo de muestreo del PI [s]

// =========================================
//  VARIABLES GLOBALES - HALLS / VELOCIDAD
// =========================================
volatile uint32_t tick_counter   = 0;   // ticks totales
uint32_t          last_ticks_sample = 0;
unsigned long     last_sample_ms  = 0;

uint8_t last_hall_state = 0;

float rpm_meas = 0.0f;
float v_ms     = 0.0f;

// =========================================
//  VARIABLES GLOBALES - CONTROL PI
// =========================================
bool  controlActivo = false;
float rpm_sp        = 0.0f;   // setpoint RPM

// Ganancias PI (ajustables)
float Kp = 0.05f;
float Ki = 0.10f;

// Estado interno del PI
float e_int   = 0.0f;
float u_cmd   = 0.0f;       // comando lógico 0–255

// Offset para salir de zona muerta del driver (~0.8–1 V)
const float DAC_OFFSET = 95.0f;   // 95/255 * 3.3 ≈ 1.23 V

// =========================================
//  FUNCIONES AUXILIARES
// =========================================

// Lee H1,H2,H3, detecta cambio de patrón y suma ticks
void actualizarTicksHall()
{
  uint8_t h1 = digitalRead(H1_PIN);
  uint8_t h2 = digitalRead(H2_PIN);
  uint8_t h3 = digitalRead(H3_PIN);

  uint8_t state = (h1 << 2) | (h2 << 1) | h3;

  if (state != last_hall_state)
  {
    last_hall_state = state;

    // Válidos típicos: 001,010,011,100,101,110
    if (state != 0b000 && state != 0b111)
    {
      tick_counter++;
    }
  }
}

// Aplica el valor u_cmd (0–255) al DAC
void aplicarDAC()
{
  if (u_cmd < 0.0f)   u_cmd = 0.0f;
  if (u_cmd > 255.0f) u_cmd = 255.0f;

  uint8_t val = (uint8_t)roundf(u_cmd);
  dacWrite(DAC1_PIN, val);
}

// Resetea el integrador y la salida
void resetPI()
{
  e_int = 0.0f;
  u_cmd = 0.0f;
  aplicarDAC();
}

// =========================================
//  SETUP
// =========================================
void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println(F("=== CONTROL PI VELOCIDAD - SALIDA DAC ==="));
  Serial.println(F("Comandos (sin espacios):"));
  Serial.println(F("  S###  -> Setpoint RPM y activa PI (ej: S400)"));
  Serial.println(F("  S0    -> Apaga PI y pone 0 V"));
  Serial.println(F("  U###  -> Salida DAC manual 0–255 (ej: U150)"));
  Serial.println();

  // Pines dirección (ajusta si tu placa necesita otra lógica)
  pinMode(REVERSA1_PIN, OUTPUT);
  pinMode(REVERSA2_PIN, OUTPUT);
  digitalWrite(REVERSA1_PIN, LOW);
  digitalWrite(REVERSA2_PIN, LOW);

  // Pines Hall
  pinMode(H1_PIN, INPUT);
  pinMode(H2_PIN, INPUT);
  pinMode(H3_PIN, INPUT);

  // DAC: no necesita modo especial, dacWrite se encarga
  u_cmd = 0.0f;
  aplicarDAC();

  last_sample_ms    = millis();
  last_ticks_sample = tick_counter;
}

// =========================================
//  LOOP
// =========================================
void loop()
{
  // ---- 1) Comandos por serie ----
  if (Serial.available())
  {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.length() > 0)
    {
      char c = linea[0];

      if (c == 'S' || c == 's')
      {
        float sp = linea.substring(1).toFloat();
        if (sp <= 0.0f)
        {
          controlActivo = false;
          rpm_sp        = 0.0f;
          resetPI();
          Serial.println(F("PI desactivado, motor parado (DAC=0)."));
        }
        else
        {
          rpm_sp        = sp;
          controlActivo = true;
          resetPI();
          Serial.print(F("Nuevo setpoint RPM = "));
          Serial.println(rpm_sp);
        }
      }
      else if (c == 'U' || c == 'u')
      {
        int val = linea.substring(1).toInt();
        if (val < 0)   val = 0;
        if (val > 255) val = 255;

        controlActivo = false;
        rpm_sp        = 0.0f;
        e_int         = 0.0f;

        u_cmd = (float)val;
        aplicarDAC();

        Serial.print(F("DAC manual = "));
        Serial.print(val);
        Serial.print(F(" -> ~"));
        Serial.print(3.3f * val / 255.0f, 2);
        Serial.println(F(" V"));
      }
    }
  }

  // ---- 2) Actualizar ticks de Hall ----
  actualizarTicksHall();

  // ---- 3) Cada WINDOW_MS, calcular RPM, velocidad y PI ----
  unsigned long now = millis();
  if (now - last_sample_ms >= WINDOW_MS)
  {
    unsigned long dt_ms = now - last_sample_ms;
    last_sample_ms      = now;

    uint32_t ticks_win = tick_counter - last_ticks_sample;
    last_ticks_sample   = tick_counter;

    // RPM
    float revs_win = (float)ticks_win / (float)TICKS_PER_REV;
    rpm_meas       = (revs_win * 60000.0f) / (float)dt_ms;

    // Velocidad lineal [m/s]
    v_ms = (rpm_meas / 60.0f) * CIRC_M;

    // ----- CONTROL PI -----
    if (controlActivo && rpm_sp > 0.0f)
    {
      float e = rpm_sp - rpm_meas;

      e_int += e * Ts;
      if (e_int > 5000.0f)  e_int = 5000.0f;
      if (e_int < -5000.0f) e_int = -5000.0f;

      float u = DAC_OFFSET + Kp * e + Ki * e_int;

      if (u < 0.0f)   u = 0.0f;
      if (u > 255.0f) u = 255.0f;

      u_cmd = u;
    }
    // si controlActivo==false, u_cmd se mantiene en el valor manual o 0

    aplicarDAC();

    // ---- Imprimir estado ----
    Serial.print(F("SP="));
    Serial.print(rpm_sp, 1);
    Serial.print(F("  RPM="));
    Serial.print(rpm_meas, 2);
    Serial.print(F("  DAC="));
    Serial.print(u_cmd, 1);
    Serial.print(F("  ticks="));
    Serial.print(ticks_win);
    Serial.print(F("  v="));
    Serial.print(v_ms, 3);
    Serial.println(F(" m/s"));
  }
}
