#include <arduinoFFT.h>

// ======================================
// CONFIGURACION
// ======================================

#define SAMPLES 256
#define SAMPLING_FREQUENCY 4000

const int micPin = 34;
const int ledPin = 2;

// ======================================
// FFT
// ======================================

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(
  vReal,
  vImag,
  SAMPLES,
  SAMPLING_FREQUENCY
);

// ======================================
// VARIABLES
// ======================================

unsigned long samplingPeriodUs;

double peakFrequency = 0;
double peakMagnitude = 0;

double signalLevel = 0;

double maxValue = -99999;
double minValue = 99999;

// ======================================
// SETUP
// ======================================

void setup() {

  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);

  analogReadResolution(12);

  samplingPeriodUs =
    round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  Serial.println();
  Serial.println("======================================");
  Serial.println(" ESP32 FFT AUDIO ANALYZER ");
  Serial.println("======================================");
}

// ======================================
// LOOP
// ======================================

void loop() {

  signalLevel = 0;

  maxValue = -99999;
  minValue = 99999;

  // ======================================
  // CAPTURA
  // ======================================

  for (int i = 0; i < SAMPLES; i++) {

    unsigned long t = micros();

    int raw = analogRead(micPin);

    double centered = raw - 2048;

    vReal[i] = centered;
    vImag[i] = 0;

    signalLevel += abs(centered);

    if (centered > maxValue)
      maxValue = centered;

    if (centered < minValue)
      minValue = centered;

    while (micros() - t < samplingPeriodUs) {
    }
  }

  // ======================================
  // NIVEL PROMEDIO
  // ======================================

  signalLevel /= SAMPLES;

  // ======================================
  // FFT
  // ======================================

  FFT.windowing(
    FFTWindow::Hamming,
    FFTDirection::Forward
  );

  FFT.compute(FFTDirection::Forward);

  FFT.complexToMagnitude();

  // ======================================
  // BUSCAR FRECUENCIA DOMINANTE
  // ======================================

  peakMagnitude = 0;
  peakFrequency = 0;

  for (int i = 1; i < SAMPLES / 2; i++) {

    if (vReal[i] > peakMagnitude) {

      peakMagnitude = vReal[i];

      peakFrequency =
        (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }

  // ======================================
  // DETECTAR SOBRECARGA
  // ======================================

  bool overload = false;

  if (
      maxValue > 1800 ||
      minValue < -1800 ||
      signalLevel > 500
     ) {

    overload = true;
  }

  // ======================================
  // LED
  // ======================================

  digitalWrite(ledPin, overload);

  // ======================================
  // TABLA SERIAL
  // ======================================

  Serial.println();
  Serial.println("======================================");

  Serial.print("Frecuencia dominante: ");
  Serial.print(peakFrequency);
  Serial.println(" Hz");

  Serial.print("Magnitud FFT: ");
  Serial.println(peakMagnitude);

  Serial.print("Nivel promedio: ");
  Serial.println(signalLevel);

  Serial.print("Valor maximo: ");
  Serial.println(maxValue);

  Serial.print("Valor minimo: ");
  Serial.println(minValue);

  Serial.print("Rango dinamico: ");
  Serial.println(maxValue - minValue);

  Serial.print("Estado LED: ");

  if (overload) {

    Serial.println("ENCENDIDO");

  } else {

    Serial.println("APAGADO");
  }

  // ======================================
  // ALERTA
  // ======================================

  if (overload) {

    Serial.println();
    Serial.println("!!! ALERTA: SOBRECARGA DETECTADA !!!");
  }

  Serial.println("======================================");

  delay(3000);
}