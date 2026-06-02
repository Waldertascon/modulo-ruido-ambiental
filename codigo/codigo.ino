#include <WiFi.h>
#include <WebServer.h>
#include <arduinoFFT.h>

#define SAMPLES 256
#define SAMPLING_FREQUENCY 4000

const char* ssid     = "AudioAnalyzer";
const char* password = "12345678";

const int micPin = 34;
const int ledPin = 2;

WebServer server(80);

double vReal[SAMPLES];
double vImag[SAMPLES];
double waveBuffer[SAMPLES];
double fftBuffer[SAMPLES / 2];

ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

double rms           = 0;
double peakFrequency = 0;
double peakMagnitude = 0;

int peakMax      = 0;
int peakMin      = 4095;
int dynamicRange = 0;

bool ledState  = false;
bool saturation = false;

int adcOffset = 2048;

// ================================
// CALIBRACION OFFSET
// ================================

void calibrateOffset() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += analogRead(micPin);
    delay(2);
  }
  adcOffset = sum / 500;
  Serial.print("Offset ADC: ");
  Serial.println(adcOffset);
}

// ================================
// CAPTURA + FFT
// ================================

void processAudio() {
  peakMax    = 0;
  peakMin    = 4095;
  saturation = false;

  unsigned long samplingPeriodUs = 1000000UL / SAMPLING_FREQUENCY;
  double rmsAccumulator = 0;

  for (int i = 0; i < SAMPLES; i++) {
    unsigned long t = micros();

    int raw = analogRead(micPin);

    if (raw > peakMax) peakMax = raw;
    if (raw < peakMin) peakMin = raw;
    if (raw > 4000 || raw < 50) saturation = true;

    double centered = raw - adcOffset;

    waveBuffer[i] = raw;
    vReal[i]      = centered;
    vImag[i]      = 0;

    rmsAccumulator += centered * centered;

    while ((micros() - t) < samplingPeriodUs);
  }

  rms          = sqrt(rmsAccumulator / SAMPLES);
  dynamicRange = peakMax - peakMin;
  ledState     = rms > 100;

  digitalWrite(ledPin, ledState);

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  for (int i = 0; i < SAMPLES / 2; i++) {
    fftBuffer[i] = vReal[i];
  }

  peakMagnitude = 0;
  peakFrequency = 0;

  for (int i = 2; i < SAMPLES / 2; i++) {
    if (vReal[i] > peakMagnitude) {
      peakMagnitude = vReal[i];
      peakFrequency = ((double)i * SAMPLING_FREQUENCY) / SAMPLES;
    }
  }

  // Umbral de ruido: si la magnitud pico o el RMS son demasiado bajos
  // (interferencia 60Hz de la red electrica, ruido ADC, microfono desconectado),
  // reportar 0 Hz en lugar de un falso positivo.
  // Ajustar NOISE_FLOOR si hace falta segun el microfono.
  const double NOISE_FLOOR = 500.0;
  const double RMS_FLOOR   = 30.0;

  if (peakMagnitude < NOISE_FLOOR || rms < RMS_FLOOR) {
    peakFrequency = 0;
    peakMagnitude = 0;
  }
}

// ================================
// JSON - METRICAS
// ================================

void handleData() {
  String json = "{";
  json += "\"rms\":"      + String(rms, 2)           + ",";
  json += "\"freq\":"     + String(peakFrequency, 2) + ",";
  json += "\"fftMag\":"   + String(peakMagnitude, 2) + ",";
  json += "\"peakMax\":"  + String(peakMax)           + ",";
  json += "\"peakMin\":"  + String(peakMin)           + ",";
  json += "\"range\":"    + String(dynamicRange)      + ",";
  json += "\"led\":"      + String(ledState   ? "true" : "false") + ",";
  json += "\"sat\":"      + String(saturation ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

// ================================
// JSON - FORMA DE ONDA
// ================================

void handleWave() {
  String json = "[";
  for (int i = 0; i < SAMPLES; i++) {
    json += String((int)waveBuffer[i]);
    if (i < SAMPLES - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// ================================
// JSON - FFT
// ================================

void handleFFT() {
  String json = "[";
  for (int i = 0; i < SAMPLES / 2; i++) {
    json += String((int)fftBuffer[i]);
    if (i < (SAMPLES / 2) - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// ================================
// JSON - TODO EN UNO (wave + fft + metricas)
// ================================

void handleTodo() {
  processAudio();  // Captura justo cuando el cliente pide datos

  String json = "{";

  // Metricas
  json += "\"rms\":"      + String(rms, 2)           + ",";
  json += "\"freq\":"     + String(peakFrequency, 2) + ",";
  json += "\"fftMag\":"   + String(peakMagnitude, 2) + ",";
  json += "\"peakMax\":"  + String(peakMax)           + ",";
  json += "\"peakMin\":"  + String(peakMin)           + ",";
  json += "\"range\":"    + String(dynamicRange)      + ",";
  json += "\"led\":"      + String(ledState    ? "true" : "false") + ",";
  json += "\"sat\":"      + String(saturation  ? "true" : "false") + ",";

  // Forma de onda
  json += "\"wave\":[";
  for (int i = 0; i < SAMPLES; i++) {
    json += String((int)waveBuffer[i]);
    if (i < SAMPLES - 1) json += ",";
  }
  json += "],";

  // Espectro FFT
  json += "\"fft\":[";
  for (int i = 0; i < SAMPLES / 2; i++) {
    json += String((int)fftBuffer[i]);
    if (i < (SAMPLES / 2) - 1) json += ",";
  }
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

// ================================
// PAGINA WEB PRINCIPAL
// ================================

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Audio Analyzer ESP32</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0d0d0d;
      color: #00ff88;
      font-family: 'Courier New', monospace;
      padding: 16px;
    }
    h1 {
      text-align: center;
      font-size: 1.4em;
      letter-spacing: 3px;
      margin-bottom: 16px;
      color: #00ffcc;
      text-shadow: 0 0 8px #00ffcc;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-bottom: 16px;
    }
    .card {
      background: #111;
      border: 1px solid #00ff8833;
      border-radius: 8px;
      padding: 12px;
    }
    .card h2 {
      font-size: 0.75em;
      letter-spacing: 2px;
      color: #00ffcc88;
      margin-bottom: 10px;
    }
    table { width: 100%; border-collapse: collapse; }
    td {
      padding: 5px 4px;
      font-size: 0.85em;
      border-bottom: 1px solid #1a1a1a;
    }
    td:last-child {
      text-align: right;
      color: #ffffff;
      font-weight: bold;
    }
    .label { color: #00ff8899; }
    canvas {
      width: 100%;
      height: 130px;
      display: block;
      background: #050505;
      border-radius: 4px;
    }
    .full { grid-column: 1 / -1; }
    #quality {
      display: inline-block;
      padding: 2px 10px;
      border-radius: 4px;
      font-size: 0.9em;
    }
    .q-mala      { color: #ff4444; }
    .q-regular   { color: #ffaa00; }
    .q-buena     { color: #88ff44; }
    .q-excelente { color: #00ffcc; text-shadow: 0 0 6px #00ffcc; }
    .sat-si { color: #ff4444; font-weight: bold; }
    .sat-no { color: #00ff88; }
    .led-on  { color: #ffff00; text-shadow: 0 0 6px #ffff00; }
    .led-off { color: #555; }
  </style>
</head>
<body>

<h1>&#9642; AUDIO ANALYZER ESP32</h1>

<div class="grid">

  <!-- METRICAS -->
  <div class="card">
    <h2>METRICAS</h2>
    <table>
      <tr><td class="label">RMS</td>        <td id="rms">--</td></tr>
      <tr><td class="label">Freq. Pico</td> <td id="freq">--</td></tr>
      <tr><td class="label">Magnitud FFT</td><td id="fftMag">--</td></tr>
      <tr><td class="label">Peak MAX</td>   <td id="peakMax">--</td></tr>
      <tr><td class="label">Peak MIN</td>   <td id="peakMin">--</td></tr>
      <tr><td class="label">Rango Din.</td> <td id="range">--</td></tr>
      <tr><td class="label">LED</td>        <td id="led">--</td></tr>
      <tr><td class="label">Saturacion</td> <td id="sat">--</td></tr>
      <tr><td class="label">Calidad</td>    <td><span id="quality">--</span></td></tr>
    </table>
  </div>

  <!-- OSCILOSCOPIO -->
  <div class="card">
    <h2>OSCILOSCOPIO</h2>
    <canvas id="scope" width="400" height="130"></canvas>
  </div>

  <!-- FFT -->
  <div class="card full">
    <h2>ESPECTRO FFT</h2>
    <canvas id="fft" width="800" height="130"></canvas>
  </div>

</div>

<script>
  const scopeCanvas = document.getElementById("scope");
  const scopeCtx   = scopeCanvas.getContext("2d");
  const fftCanvas   = document.getElementById("fft");
  const fftCtx     = fftCanvas.getContext("2d");

  // --- UN SOLO FETCH: todos los datos del mismo frame ---
  function updateAll() {
    fetch('/todo')
      .then(r => r.json())
      .then(data => {

        // --- METRICAS ---
        document.getElementById('rms').textContent     = data.rms.toFixed(1);
        document.getElementById('freq').textContent    = data.freq.toFixed(1) + " Hz";
        document.getElementById('fftMag').textContent  = data.fftMag.toFixed(1);
        document.getElementById('peakMax').textContent = data.peakMax;
        document.getElementById('peakMin').textContent = data.peakMin;
        document.getElementById('range').textContent   = data.range;

        const ledEl = document.getElementById('led');
        ledEl.textContent = data.led ? "ON" : "OFF";
        ledEl.className   = data.led ? "led-on" : "led-off";

        const satEl = document.getElementById('sat');
        satEl.textContent = data.sat ? "SI" : "NO";
        satEl.className   = data.sat ? "sat-si" : "sat-no";

        let calidad = "MALA";
        let clase   = "q-mala";
        if (data.rms > 50)  { calidad = "REGULAR";   clase = "q-regular"; }
        if (data.rms > 100) { calidad = "BUENA";      clase = "q-buena"; }
        if (data.rms > 200) { calidad = "EXCELENTE";  clase = "q-excelente"; }
        const qEl = document.getElementById('quality');
        qEl.textContent = calidad;
        qEl.className   = clase;

        // --- OSCILOSCOPIO ---
        const wave = data.wave;
        const sw = scopeCanvas.width;
        const sh = scopeCanvas.height;

        scopeCtx.clearRect(0, 0, sw, sh);
        scopeCtx.strokeStyle = "#1a1a1a";
        scopeCtx.lineWidth = 1;
        for (let g = 0; g <= 4; g++) {
          let gy = g * sh / 4;
          scopeCtx.beginPath();
          scopeCtx.moveTo(0, gy);
          scopeCtx.lineTo(sw, gy);
          scopeCtx.stroke();
        }
        scopeCtx.strokeStyle = "#00ff88";
        scopeCtx.lineWidth = 1.5;
        scopeCtx.shadowColor = "#00ff88";
        scopeCtx.shadowBlur = 4;
        scopeCtx.beginPath();
        for (let i = 0; i < wave.length; i++) {
          let x = i * sw / wave.length;
          let y = sh - (wave[i] / 4095.0) * sh;
          if (i === 0) scopeCtx.moveTo(x, y);
          else         scopeCtx.lineTo(x, y);
        }
        scopeCtx.stroke();
        scopeCtx.shadowBlur = 0;

        // --- FFT ---
        const fft = data.fft;
        const fw = fftCanvas.width;
        const fh = fftCanvas.height;

        fftCtx.clearRect(0, 0, fw, fh);
        const bars = fft.length;
        const bw   = fw / bars;

        let maxMag = 1;
        for (let i = 2; i < bars; i++) {
          if (fft[i] > maxMag) maxMag = fft[i];
        }
        for (let i = 2; i < bars; i++) {
          let mag  = fft[i];
          let barH = (mag / maxMag) * fh;
          let ratio = mag / maxMag;
          let r = Math.floor(ratio * 255);
          let g = Math.floor((1 - ratio * 0.5) * 255);
          fftCtx.fillStyle = "rgb(" + r + "," + g + ",80)";
          fftCtx.fillRect(i * bw, fh - barH, Math.max(bw - 1, 1), barH);
        }
      })
      .catch(() => {});
  }

  setInterval(updateAll, 250);
  updateAll();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// ================================
// SETUP
// ================================

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  analogReadResolution(12);

  Serial.println();
  Serial.println("Iniciando...");

  calibrateOffset();

  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",      handleRoot);
  server.on("/datos", handleData);
  server.on("/wave",  handleWave);
  server.on("/fft",   handleFFT);
  server.on("/todo",  handleTodo);

  server.begin();
  Serial.println("Servidor listo");
}

// ================================
// LOOP
// ================================

void loop() {
  server.handleClient();
}
