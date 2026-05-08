#include <Arduino.h>

// ================= CONFIG =================
#define MIC_PIN 39          // VP / GPIO36
#define ADC_BITS 12

// filtro simple
float filtered = 0;
float alpha = 0.15;   // suavizado

// referencia DC
float dcOffset = 2048;

void setup() {

  Serial.begin(115200);

  analogReadResolution(ADC_BITS);
  analogSetAttenuation(ADC_11db);

  delay(500);

  Serial.println("Sistema iniciado");
}

void loop() {

  // ===== LEER ADC =====
  int raw = analogRead(MIC_PIN);

  // ===== REMOVER OFFSET =====
  dcOffset = dcOffset + 0.001 * (raw - dcOffset);

  float signal = raw - dcOffset;

  // ===== FILTRO PASA BAJOS SIMPLE =====
  filtered = alpha * signal + (1 - alpha) * filtered;

  // ===== MOSTRAR =====

  // Para Serial Plotter
  Serial.print(signal);
  Serial.print("\t");
  Serial.println(filtered);

  delay(5);
}