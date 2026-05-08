#include <Arduino.h>
#include <arduinoFFT.h>

// ================= CONFIG =================
#define MIC_PIN          36
#define SAMPLES          256
#define SAMPLE_RATE      8000
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE)

#define ADC_MAX          4095.0f
#define VREF             3.3f

#define PRINT_INTERVAL   1200

// MODOS
#define MODE_ANALYSIS    1
#define MODE_PLOTTER     0
#define MODE_DEBUG       0

// ================= FFT =================
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLE_RATE);

// ================= ESTADO =================
float dcEstimate = 2048.0f;
float prevSample = 0;
uint32_t lastPrint = 0;

// ================= FUNCIONES =================

// Filtro simple (más estable que biquad si hay problemas)
float simpleLPF(float input) {
  float alpha = 0.2;
  prevSample = alpha * input + (1 - alpha) * prevSample;
  return prevSample;
}

// Detectar saturación
bool isSaturated(int raw) {
  return (raw < 10 || raw > 4085);
}

// Detectar señal inválida (muy baja variación)
bool isFlatSignal(float rms) {
  return rms < 2.0;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  delay(500);
  Serial.println("\n[ESP32] Sistema listo - GPIO36 (VP)");
}

// ================= LOOP =================
void loop() {

  uint32_t t0 = micros();

  int satCount = 0;
  double sumSq = 0;

  // ===== MUESTREO =====
  for (int i = 0; i < SAMPLES; i++) {

    int raw = analogRead(MIC_PIN);

    if (isSaturated(raw)) satCount++;

    float sig = (float)raw;

    // eliminar DC dinámico
    dcEstimate += 0.001 * (sig - dcEstimate);
    sig -= dcEstimate;

    // filtro suave
    sig = simpleLPF(sig);

    vReal[i] = sig;
    vImag[i] = 0;

    sumSq += sig * sig;

#if MODE_DEBUG
    Serial.println(raw);
#endif

    while ((micros() - t0) < (i + 1) * SAMPLE_PERIOD_US);
  }

  // ===== RMS =====
  float rms = sqrt(sumSq / SAMPLES);
  float rmsV = rms * (VREF / ADC_MAX);

  float dBFS = (rms > 1.0) ? 20.0 * log10(rms / 2048.0) : -80.0;

  // ===== FFT =====
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  float domFreq = 0;
  float maxMag = 0;

  for (int i = 5; i < SAMPLES / 2; i++) { // ignorar DC y ruido bajo
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      domFreq = (i * SAMPLE_RATE) / SAMPLES;
    }
  }

  // ===== DIAGNÓSTICO =====
  bool saturated = (satCount > SAMPLES * 0.2);
  bool flat      = isFlatSignal(rms);

#if MODE_PLOTTER
  // Solo 2 señales claras
  Serial.print(rms);
  Serial.print("\t");
  Serial.println(domFreq);
#endif

#if MODE_ANALYSIS
  if (millis() - lastPrint > PRINT_INTERVAL) {
    lastPrint = millis();

    Serial.println("\n================ ANALISIS ================");

    Serial.print("RMS: ");
    Serial.println(rms);

    Serial.print("Voltaje RMS: ");
    Serial.print(rmsV, 4);
    Serial.println(" V");

    Serial.print("Nivel (dBFS): ");
    Serial.println(dBFS);

    Serial.print("Frecuencia dominante: ");
    Serial.print(domFreq);
    Serial.println(" Hz");

    // ===== ALERTAS =====
    Serial.println("\n--- DIAGNOSTICO ---");

    if (saturated) {
      Serial.println("⚠ Señal SATURADA (ganancia muy alta o sin VREF)");
    } else if (flat) {
      Serial.println("⚠ Señal MUY BAJA o PIN FLOTANTE");
    } else {
      Serial.println("✔ Señal valida");
    }

    Serial.println("==========================================");
  }
#endif
}