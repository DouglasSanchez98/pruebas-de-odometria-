// ===== ESP32: DAC como acelerador + Tacómetro con Halls =====

// ----- Pines -----
#define H1 32
#define H2 33
#define H3 27
#define DAC_PIN 25   // DAC1 de la ESP32 (salida analógica 0-3.3V aprox.)

// ----- Parámetros de la rueda -----
const double CIRC_M = 0.7435;      // perímetro de la rueda [m]
const int    CPR    = 90;          // ticks por vuelta (6 estados * 15 pares de polos)

// Anti-ruido y ventana de cálculo
const unsigned long MIN_TICK_US = 150;   // mínimo entre flancos [us]
const unsigned long WINDOW_MS   = 250;   // ventana para cálculo de RPM [ms]

// ----- Variables de los Halls (compartidas con la ISR) -----
volatile uint8_t       last_state = 0xFF;
volatile unsigned long tick_count = 0;

// ----- Variables para el DAC -----
uint8_t dac_value = 0;   // 0..255

// Lee estado de los tres Halls (H1H2H3 -> 3 bits)
inline uint8_t read_state() {
  uint8_t b1 = digitalRead(H1);
  uint8_t b2 = digitalRead(H2);
  uint8_t b3 = digitalRead(H3);
  return (uint8_t)((b1 << 2) | (b2 << 1) | b3);
}

// ISR: cuenta ticks válidos
void IRAM_ATTR hall_isr() {
  static unsigned long last_tick_us = 0;

  uint8_t s = read_state();
  if (s == 0b000 || s == 0b111) return;   // estados “raros”

  unsigned long now = micros();

  if (last_state == 0xFF) {
    last_state   = s;
    last_tick_us = now;
    return;
  }

  if (s != last_state) {
    unsigned long dt = now - last_tick_us;
    if (dt >= MIN_TICK_US) {
      tick_count++;          // un tick más en la ventana
      last_tick_us = now;
      last_state   = s;
    }
  }
}

// ----- Lectura de número desde el Monitor Serie -----
void checkSerialForDAC() {
  static String buffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        int val = buffer.toInt();
        if (val < 0)   val = 0;
        if (val > 255) val = 255;
        dac_value = (uint8_t)val;
        dacWrite(DAC_PIN, dac_value);
        Serial.print("Nuevo DAC = ");
        Serial.print(dac_value);
        Serial.print("  (V aprox = ");
        Serial.print((3.3 * dac_value) / 255.0, 2);
        Serial.println(" V)");
      }
      buffer = "";
    } else {
      if (isDigit(c)) buffer += c;
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

  // Inicializamos el DAC en 0 (sin acelerar)
  dacWrite(DAC_PIN, dac_value);

  Serial.println("DAC + Tacómetro listo.");
  Serial.println("Escribe un valor 0-255 en el Monitor Serie y pulsa Enter.");
}

void loop() {
  // 1) Revisar si escribiste un nuevo valor para el DAC
  checkSerialForDAC();

  // 2) Cada WINDOW_MS ms, calcular RPM y velocidad
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
      rpm  = ( (double)delta_ticks * 60.0 ) / ( (double)CPR * T );
      v_ms = (CIRC_M * rpm) / 60.0;
    }

    Serial.print("DAC = ");
    Serial.print(dac_value);
    Serial.print("  ticks ventana = ");
    Serial.print(delta_ticks);
    Serial.print("  RPM = ");
    Serial.print(rpm, 2);
    Serial.print("  v[m/s] = ");
    Serial.println(v_ms, 3);
  }
}
