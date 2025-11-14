// ===== TACÓMETRO SUAVIZADO POR VENTANA DE TIEMPO =====

#define H1 32
#define H2 33
#define H3 27

const double CIRC_M = 0.7435;      // perímetro [m]
const int    CPR    = 90;          // ticks por vuelta
const unsigned long MIN_TICK_US = 150;

// ventana de cálculo de velocidad [ms]
const unsigned long WINDOW_MS = 250;

// ISR: solo cuenta ticks válidos
volatile uint8_t  last_state = 0xFF;
volatile unsigned long tick_count = 0;

inline uint8_t read_state() {
  uint8_t b1 = digitalRead(H1);
  uint8_t b2 = digitalRead(H2);
  uint8_t b3 = digitalRead(H3);
  return (uint8_t)((b1<<2)|(b2<<1)|b3);
}

void IRAM_ATTR hall_isr() {
  static unsigned long last_tick_us = 0;

  uint8_t s = read_state();
  if (s == 0b000 || s == 0b111) return;

  unsigned long now = micros();
  if (last_state == 0xFF) {
    last_state   = s;
    last_tick_us = now;
    return;
  }

  if (s != last_state) {
    unsigned long dt = now - last_tick_us;
    if (dt >= MIN_TICK_US) {
      tick_count++;
      last_tick_us = now;
      last_state   = s;
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
  Serial.println("Tacómetro con ventana listo.");
}

void loop() {
  static unsigned long t0 = millis();
  static unsigned long last_ticks = 0;

  unsigned long now = millis();
  if (now - t0 >= WINDOW_MS) {
    t0 = now;

    unsigned long ticks_now;
    noInterrupts();
    ticks_now = tick_count;
    interrupts();

    unsigned long delta_ticks = ticks_now - last_ticks;
    last_ticks = ticks_now;

    double rpm  = 0.0;
    double v_ms = 0.0;

    if (delta_ticks > 0) {
      double T = (double)WINDOW_MS / 1000.0; // segundos de ventana
      // rpm = (vueltas/seg) * 60 = (ticks/(CPR*T))*60
      rpm  = ( (double)delta_ticks * 60.0 ) / ( (double)CPR * T );
      v_ms = (CIRC_M * rpm) / 60.0;
    }

    Serial.print("ticks ventana = ");
    Serial.print(delta_ticks);
    Serial.print("   RPM = ");
    Serial.print(rpm, 2);
    Serial.print("   v[m/s] = ");
    Serial.println(v_ms, 3);
  }
}

