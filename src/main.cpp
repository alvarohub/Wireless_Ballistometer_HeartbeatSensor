// =====================================================================
// HeartBeat Accelerometer Sensor — M5Stack AtomS3R
// =====================================================================
//
// Reads the BMI270 IMU (accelerometer) at a configurable sample rate,
// buffers data in a lock-free ring buffer (FreeRTOS task on Core 0),
// then sends it in JSON batches over WebSocket to a browser-based UI.
//
// WiFi mode : Access Point  (HeartBeat_Sensor / heartbeat)
// Web UI    : http://192.168.4.1
// WebSocket : ws://192.168.4.1:81
// Serial    : 115200 baud (USB-CDC)
// =====================================================================

#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include "ring_buffer.h"
#include "signal_processing.h"
#include "web_content.h"

// ======================== CONFIGURATION ==============================

static const char* AP_SSID = "HeartBeat_Sensor";
static const char* AP_PASS = "heartbeat";   // min 8 chars (WPA2)

// ======================== GLOBALS ====================================

WebServer       httpServer(80);
WebSocketsServer wsServer(81);

// Ring buffer: 1024 entries ≈ 10 s @ 100 Hz, ~2 s @ 500 Hz
RingBuffer<AccelSample, 1024> sampleBuffer;

SignalProcessor sigProc;

// Sampling parameters (modified from WebSocket commands)
volatile bool     samplingActive  = false;
volatile uint16_t sampleRateHz    = 100;
volatile uint16_t sampleDurationS = 0;     // 0 = continuous
uint32_t          samplingStartMs = 0;
volatile uint32_t totalSamples    = 0;

TaskHandle_t samplingTaskHandle = nullptr;

// Timing
unsigned long lastWsSendMs      = 0;
unsigned long lastDisplayMs     = 0;
const int     WS_SEND_INTERVAL  = 50;   // ms  → 20 batches/s
const int     DISPLAY_INTERVAL  = 500;  // ms

// ===================== SAMPLING TASK (Core 0) ========================
// Runs at high priority on Core 0.  Only accesses M5.Imu (I2C).
// Core 1 (Arduino loop) handles WiFi, WebSocket, display, buttons.

void samplingTask(void* /*param*/) {
    TickType_t xWake = xTaskGetTickCount();

    for (;;) {
        if (samplingActive) {
            TickType_t period = pdMS_TO_TICKS(1000 / sampleRateHz);

            if (M5.Imu.update()) {
                auto d = M5.Imu.getImuData();

                AccelSample s;
                s.timestamp_us = micros();
                s.ax = d.accel.x;
                s.ay = d.accel.y;
                s.az = d.accel.z;
                s.magnitude = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);

                sampleBuffer.push(s);
                totalSamples++;

                // Auto-stop after duration
                if (sampleDurationS > 0 &&
                    (millis() - samplingStartMs) >= (uint32_t)sampleDurationS * 1000) {
                    samplingActive = false;
                }
            }

            vTaskDelayUntil(&xWake, period);
        } else {
            // Idle — sleep 10 ms, reset timing base
            vTaskDelay(pdMS_TO_TICKS(10));
            xWake = xTaskGetTickCount();
        }
    }
}

// ===================== WEBSOCKET EVENTS ==============================

void broadcastConfig() {
    JsonDocument doc;
    doc["type"]       = "config";
    doc["sampleRate"] = sampleRateHz;
    doc["duration"]   = sampleDurationS;
    doc["sampling"]   = samplingActive;
    doc["imuOk"]      = M5.Imu.isEnabled();
    String json;
    serializeJson(doc, json);
    wsServer.broadcastTXT(json);
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {

    case WStype_CONNECTED:
        Serial.printf("[WS] Client %u connected\n", num);
        {   // Send current config to the new client
            JsonDocument doc;
            doc["type"]       = "config";
            doc["sampleRate"] = sampleRateHz;
            doc["duration"]   = sampleDurationS;
            doc["sampling"]   = samplingActive;
            doc["imuOk"]      = M5.Imu.isEnabled();
            String json;
            serializeJson(doc, json);
            wsServer.sendTXT(num, json);
        }
        break;

    case WStype_DISCONNECTED:
        Serial.printf("[WS] Client %u disconnected\n", num);
        break;

    case WStype_TEXT: {
        JsonDocument doc;
        if (deserializeJson(doc, payload, length)) break;
        const char* cmd = doc["cmd"];
        if (!cmd) break;

        if (strcmp(cmd, "start") == 0) {
            sampleBuffer.clear();
            sigProc.reset();
            sigProc.configure(sampleRateHz);
            totalSamples    = 0;
            samplingStartMs = millis();
            samplingActive  = true;
            Serial.println("[CMD] Sampling START");
        }
        else if (strcmp(cmd, "stop") == 0) {
            samplingActive = false;
            Serial.println("[CMD] Sampling STOP");
        }
        else if (strcmp(cmd, "set_rate") == 0) {
            int v = doc["value"] | 100;
            sampleRateHz = constrain(v, 10, 500);
            sigProc.configure(sampleRateHz);
            Serial.printf("[CMD] Rate → %d Hz\n", sampleRateHz);
        }
        else if (strcmp(cmd, "set_duration") == 0) {
            sampleDurationS = doc["value"] | 0;
            Serial.printf("[CMD] Duration → %d s\n", sampleDurationS);
        }
        else if (strcmp(cmd, "set_filters") == 0) {
            float hp = doc["hp"] | 0.5f;
            float lp = doc["lp"] | 10.0f;
            sigProc.configure(sampleRateHz, hp, lp);
            sigProc.reset();
            Serial.printf("[CMD] Filters → HP %.1f  LP %.1f Hz\n", hp, lp);
        }

        broadcastConfig();  // echo updated config to all clients
    } break;

    default: break;
    }
}

// =================== BATCH DATA SEND ================================
// Drains the ring buffer, runs signal processing, and sends a JSON
// batch to all connected WebSocket clients.

void processAndSendData() {
    const int MAX_BATCH = 50;  // ≤ 50 samples per JSON message
    AccelSample buf[MAX_BATCH];
    float       filtBuf[MAX_BATCH];   // filtered values to send
    int count = 0;

    // Always drain the buffer (keeps signal processing running for LCD BPM)
    while (count < MAX_BATCH && sampleBuffer.pop(buf[count])) {
        float f = sigProc.process(buf[count].magnitude);
        filtBuf[count] = f;
        sigProc.detectPeak(f, buf[count].timestamp_us);
        count++;
    }

    if (count == 0) return;
    if (!wsServer.connectedClients()) return;  // no one to send to

    // Drain detected peaks
    SignalProcessor::PeakInfo peaks[SignalProcessor::MAX_RECENT_PEAKS];
    int nPeaks = sigProc.drainPeaks(peaks, SignalProcessor::MAX_RECENT_PEAKS);

    if (nPeaks > 0) {
        Serial.printf("[PEAK] %d peaks detected! ", nPeaks);
        for (int i = 0; i < nPeaks; i++) {
            Serial.printf(" t=%lu interval=%.0fms", 
                          (unsigned long)peaks[i].timestamp_us, 
                          peaks[i].interval_ms);
        }
        Serial.printf(" BPM=%.1f\n", sigProc.getBPM());
    }

    // Debug: print filter stats periodically
    static unsigned long lastFilterDebug = 0;
    if (millis() - lastFilterDebug >= 3000 && count > 0) {
        Serial.printf("[FILT] filtered=%.6f  mean=%.6f  sigma=%.6f  AC=%.6f  thrHi=%.6f  thrLo=%.6f\n",
                      sigProc.getFiltered(), sigProc.getMean(), sigProc.getSigma(),
                      sigProc.getAC(), sigProc.getThrHi(), sigProc.getThrLo());
        lastFilterDebug = millis();
    }

    // Build JSON batch
    JsonDocument doc;
    doc["type"] = "data";
    doc["bpm"]  = sigProc.getBPM();
    doc["hrv"]  = sigProc.getHRV();

    JsonArray jt = doc["t"].to<JsonArray>();
    JsonArray jx = doc["x"].to<JsonArray>();
    JsonArray jy = doc["y"].to<JsonArray>();
    JsonArray jz = doc["z"].to<JsonArray>();
    JsonArray jm = doc["m"].to<JsonArray>();
    JsonArray jf = doc["f"].to<JsonArray>();   // filtered magnitude

    for (int i = 0; i < count; i++) {
        jt.add(buf[i].timestamp_us);
        jx.add(serialized(String(buf[i].ax, 4)));
        jy.add(serialized(String(buf[i].ay, 4)));
        jz.add(serialized(String(buf[i].az, 4)));
        jm.add(serialized(String(buf[i].magnitude, 4)));
        jf.add(serialized(String(filtBuf[i], 6)));
    }

    // Add server-detected peak timestamps (micros)
    if (nPeaks > 0) {
        JsonArray jp = doc["peaks"].to<JsonArray>();
        JsonArray ji = doc["intervals"].to<JsonArray>();
        for (int i = 0; i < nPeaks; i++) {
            jp.add(peaks[i].timestamp_us);
            ji.add(serialized(String(peaks[i].interval_ms, 1)));
        }
    }

    String json;
    serializeJson(doc, json);
    wsServer.broadcastTXT(json);
}

// =================== LCD DISPLAY ====================================

void updateDisplay() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);

    // Row 1 — SSID
    M5.Display.setTextColor(TFT_CYAN);
    M5.Display.setCursor(2, 2);
    M5.Display.print(AP_SSID);

    // Row 2 — password
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.setCursor(2, 14);
    M5.Display.printf("pw: %s", AP_PASS);

    // Row 3 — IP
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setCursor(2, 26);
    M5.Display.print(WiFi.softAPIP().toString().c_str());

    // Row 4 — IMU status
    M5.Display.setCursor(2, 42);
    if (M5.Imu.isEnabled()) {
        M5.Display.setTextColor(TFT_GREEN);
        M5.Display.printf("IMU OK  type:%d", M5.Imu.getType());
    } else {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.print("IMU FAIL");
    }

    // Row 5 — sampling status
    M5.Display.setCursor(2, 56);
    M5.Display.setTextColor(samplingActive ? TFT_GREEN : TFT_YELLOW);
    M5.Display.printf("%s %dHz  #%lu",
                      samplingActive ? "REC" : "IDLE",
                      sampleRateHz,
                      (unsigned long)totalSamples);

    // BPM — large
    float bpm = sigProc.getBPM();
    if (bpm > 0) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.setTextSize(3);
        M5.Display.setCursor(10, 78);
        M5.Display.printf("%.0f", bpm);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(90, 92);
        M5.Display.print("BPM");
    }

    // HRV — small at bottom
    float hrv = sigProc.getHRV();
    if (hrv > 0) {
        M5.Display.setTextColor(TFT_DARKGREY);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(2, 118);
        M5.Display.printf("HRV %.0f ms", hrv);
    }
}

// ======================== SETUP =====================================

void setup() {
    // ---- M5 init ----
    auto cfg = M5.config();
    M5.begin(cfg);

    Serial.begin(115200);
    delay(1000);  // give USB-CDC time to enumerate

    Serial.println();
    Serial.println("========================================");
    Serial.println("  HeartBeat Accelerometer Sensor");
    Serial.println("  Device: M5Stack AtomS3R (ESP32-S3)");
    Serial.println("========================================");

    // ---- IMU check ----
    if (M5.Imu.isEnabled()) {
        Serial.printf("[IMU] OK — type %d\n", M5.Imu.getType());
    } else {
        Serial.println("[IMU] *** NOT DETECTED ***");
        Serial.println("[IMU] Is this really an AtomS3R (with built-in IMU)?");
        // blink red
        for (int i = 0; i < 20; i++) {
            M5.Display.fillScreen(i & 1 ? TFT_RED : TFT_BLACK);
            delay(200);
        }
    }

    // ---- IMU self-test: read 10 samples ----
    Serial.println("[IMU] Self-test — 10 readings:");
    for (int i = 0; i < 10; i++) {
        if (M5.Imu.update()) {
            auto d = M5.Imu.getImuData();
            Serial.printf("  [%d] aX=%+.4f  aY=%+.4f  aZ=%+.4f g\n",
                          i, d.accel.x, d.accel.y, d.accel.z);
        } else {
            Serial.printf("  [%d] update() returned false\n", i);
        }
        delay(50);
    }

    // ---- WiFi AP ----
    Serial.printf("[WiFi] Starting AP: %s  (pw: %s)\n", AP_SSID, AP_PASS);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(200);
    Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    // ---- HTTP server ----
    httpServer.on("/", []() {
        httpServer.send(200, "text/html", WEB_PAGE);
    });
    httpServer.begin();
    Serial.println("[HTTP] Serving web UI on port 80");

    // ---- WebSocket server ----
    wsServer.begin();
    wsServer.onEvent(onWebSocketEvent);
    Serial.println("[WS]   WebSocket server on port 81");

    // ---- Signal processing ----
    sigProc.configure(sampleRateHz);

    // ---- Start sampling task on Core 0 (high priority) ----
    xTaskCreatePinnedToCore(
        samplingTask,
        "IMU_Sample",
        4096,
        nullptr,
        configMAX_PRIORITIES - 1,
        &samplingTaskHandle,
        0   // Core 0  (Arduino loop runs on Core 1)
    );
    Serial.println("[TASK] Sampling task → Core 0");

    // ---- Initial display ----
    updateDisplay();

    Serial.println();
    Serial.println("=== READY ===");
    Serial.printf("Connect to WiFi \"%s\" and open http://%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
    Serial.printf("Password: %s\n", AP_PASS);
    Serial.println("Press the button on the AtomS3R to toggle sampling.");
    Serial.println();
}

// ======================== LOOP (Core 1) ==============================

void loop() {
    M5.update();

    // ---- Network ----
    httpServer.handleClient();
    wsServer.loop();

    unsigned long now = millis();

    // ---- Batch send via WebSocket ----
    if (now - lastWsSendMs >= WS_SEND_INTERVAL) {
        processAndSendData();
        lastWsSendMs = now;
    }

    // ---- LCD ----
    if (now - lastDisplayMs >= DISPLAY_INTERVAL) {
        updateDisplay();
        lastDisplayMs = now;
    }

    // ---- Button: toggle sampling ----
    if (M5.BtnA.wasPressed()) {
        if (samplingActive) {
            samplingActive = false;
            Serial.println("[BTN] Sampling STOP");
        } else {
            sampleBuffer.clear();
            sigProc.reset();
            sigProc.configure(sampleRateHz);
            totalSamples    = 0;
            samplingStartMs = millis();
            samplingActive  = true;
            Serial.println("[BTN] Sampling START");
        }
        broadcastConfig();
    }

    // ---- Periodic serial debug ----
    static unsigned long lastSerial = 0;
    if (now - lastSerial >= 2000 && samplingActive) {
        Serial.printf("[LIVE] #%lu  buf=%d  BPM=%.1f  HRV=%.1f ms\n",
                      (unsigned long)totalSamples,
                      (int)sampleBuffer.available(),
                      sigProc.getBPM(),
                      sigProc.getHRV());
        lastSerial = now;
    }

    delay(1);
}
