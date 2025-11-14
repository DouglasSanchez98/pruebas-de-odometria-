// ===== MEDICIÓN DE RPM Y DISTANCIA CON 3 HALLS – ESP32 =====
// Conexiones asumidas:
// Rojo  (VCC Halls) -> +5V
// Negro (GND Halls) -> GND común con ESP32
// Amarillo -> H1 -> GPIO32 + pull-up 4.7k a 3.3V
// Verde    -> H2 -> GPIO33 + pull-up 4.7k a 3.3V
// Azul     -> H3 -> GPIO27 + pull-up 4.7k a 3.3V

#define H1 32
#define H2 33
#define H3 27

// Parámetros de la rueda
const double CIRC_M = 0.7435;   // circunferencia [m]
const int    CPR    = 90;       // cambios de estado por vuelta mecánica

// Anti-ruido: tiempo mínimo entre cambios válidos [us]
const unsigned long MIN_TICK_US = 150;

// Variables compartidas con la ISR
volatile unsigned long last_tick_us = 0;
volatile unsigned long dt_tick_us   = 0;
volatile long          tick_count   = 0;
volatile uint8_t       last_state   = 0xFF;

// Lee el estado 3-bit actual H1H2H3
inline uint8_t read_state() {
  uint8_t b1 = digitalRead(H1);
  uint8_t b2 = digitalRead(H2);
  uint8_t b3 = digitalRead(H3);
  return (uint8_t)((b1<<2) | (b2<<1) | b3);
}

void IRAM_ATTR hall_isr() {
  uint8_t s = read_state();

  // ignoramos estados 000 y 111 (ruido, borde raro)
  if (s == 0b000 || s == 0b111) return;

  unsigned long now = micros();

  if (last_state == 0xFF) {
    // primer estado válido
    last_state   = s;
    last_tick_us = now;
    return;
  }

  if (s != last_state) {
    unsigned long dt = now - last_tick_us;
    if (dt >= MIN_TICK_US) {
      dt_tick_us   = dt;
      last_tick_us = now;
      last_state   = s;
      tick_count++;          // aquí solo contamos magnitud (sin signo)
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(H1, INPUT);
  pinMode(H2, INPUT);
  pinMode(H3, INPUT);

  attachInterrupt(digitalPinToInterrupt(H1), hall_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2), hall_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H3), hall_isr, CHANGE);

  Serial.println("RPM + distancia con Halls lista. Gira la rueda...");
}

void loop() {
  static unsigned long t0 = 0;
  if (millis() - t0 >= 200) {   // refresco cada 200 ms
    t0 = millis();

    long ticks_copy;
    unsigned long dt_us_copy;

    noInterrupts();
    ticks_copy  = tick_count;
    dt_us_copy  = dt_tick_us;
    interrupts();

    // RPM (magnitud) a partir del último periodo entre cambios
    double rpm = 0.0;
    if (dt_us_copy > 0) {
      rpm = (60.0 * 1e6) / (CPR * (double)dt_us_copy);
    }

    // Distancia recorrida [m] (sin signo)
    double distance_m = ( (double)ticks_copy * CIRC_M ) / CPR;

    // Velocidad lineal [m/s]
    double v_ms = (CIRC_M * rpm) / 60.0;

    Serial.print("ticks=");
    Serial.print(ticks_copy);
    Serial.print("  rpm=");
    Serial.print(rpm, 2);
    Serial.print("  v[m/s]=");
    Serial.print(v_ms, 3);
    Serial.print("  s[m]=");
    Serial.print(distance_m, 4);
    Serial.print("  res[mm/tick]=");
    Serial.println((CIRC_M/CPR)*1000.0, 3);
  }
}
