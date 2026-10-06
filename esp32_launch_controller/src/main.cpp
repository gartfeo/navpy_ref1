#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// ============== CONFIGURATION ==============
const char* WIFI_SSID = "garts25";
const char* WIFI_PASSWORD = "arthur2019";
const char* HOSTNAME = "launch-controller";  // Access via http://launch-controller.local/

// GPIO pins for each launch channel (directly control MOSFET gates)
const int CHANNEL_PINS[] = {16, 17, 18, 19, 21, 22};
const int NUM_CHANNELS = 6;

// ============== GLOBALS ==============
WebServer server(80);
int activeChannel = -1;  // Active launch channel (-1 = none)

// ============== FUNCTION DECLARATIONS ==============
void handleRoot();
void handleHealth();
void handleLaunch();
void handleStatus();
void handleReset();
void handleNotFound();

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Launch Controller ===");

    // Initialize all channel pins as OUTPUT, LOW (off)
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pinMode(CHANNEL_PINS[i], OUTPUT);
        digitalWrite(CHANNEL_PINS[i], LOW);
    }

    // Connect to WiFi
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    // Setup mDNS hostname
    if (MDNS.begin(HOSTNAME)) {
        Serial.printf("mDNS started: http://%s.local/\n", HOSTNAME);
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("mDNS failed to start");
    }

    // Setup HTTP endpoints
    server.on("/", handleRoot);
    server.on("/health", handleHealth);
    server.on("/launch", handleLaunch);
    server.on("/status", handleStatus);
    server.on("/reset", handleReset);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("HTTP server started");
}

// ============== LOOP ==============
void loop() {
    server.handleClient();
}

// ============== HTTP HANDLERS ==============

void handleRoot() {
    String html = "<html><head><title>ESP32 Launch Controller</title></head><body>";
    html += "<h1>ESP32 Launch Controller</h1>";
    html += "<p>Endpoints:</p><ul>";
    html += "<li>GET /health - Check connectivity</li>";
    html += "<li>GET /launch?channel=N - Trigger channel N (1-" + String(NUM_CHANNELS) + ")</li>";
    html += "<li>GET /status - Get system status</li>";
    html += "<li>GET /reset - Reset all channels to OFF</li>";
    html += "</ul>";
    html += "<h2>Manual Trigger</h2>";
    for (int i = 1; i <= NUM_CHANNELS; i++) {
        html += "<a href='/launch?channel=" + String(i) + "'><button>Launch Ch " + String(i) + "</button></a> ";
    }
    html += "<br><br><a href='/reset'><button style='background:red;color:white;'>Reset All</button></a>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleHealth() {
    server.send(200, "text/plain", "OK");
}

void handleLaunch() {
    // Check if channel parameter is provided
    if (!server.hasArg("channel")) {
        server.send(400, "application/json", "{\"error\":\"Missing channel parameter\"}");
        return;
    }

    int channel = server.arg("channel").toInt();

    // Validate channel number (1-indexed for user, 0-indexed internally)
    if (channel < 1 || channel > NUM_CHANNELS) {
        String error = "{\"error\":\"Invalid channel. Must be 1-" + String(NUM_CHANNELS) + "\"}";
        server.send(400, "application/json", error);
        return;
    }

    int pinIndex = channel - 1;  // Convert to 0-indexed
    int pin = CHANNEL_PINS[pinIndex];

    Serial.printf("Triggering channel %d (GPIO %d)\n", channel, pin);

    // Trigger the channel
    activeChannel = channel;
    digitalWrite(pin, HIGH);

    // Send response - channel stays on until /reset is called
    StaticJsonDocument<128> doc;
    doc["success"] = true;
    doc["channel"] = channel;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);

    Serial.printf("Channel %d is ON (use /reset to turn off)\n", channel);
}

void handleStatus() {
    StaticJsonDocument<256> doc;

    // List available channels
    JsonArray channels = doc.createNestedArray("channels");
    for (int i = 1; i <= NUM_CHANNELS; i++) {
        channels.add(i);
    }

    // Current active channel
    if (activeChannel > 0) {
        doc["active"] = activeChannel;
    } else {
        doc["active"] = nullptr;
    }

    // WiFi info
    doc["ip"] = WiFi.localIP().toString();
    doc["hostname"] = String(HOSTNAME) + ".local";
    doc["rssi"] = WiFi.RSSI();

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleReset() {
    // Turn off all channels
    for (int i = 0; i < NUM_CHANNELS; i++) {
        digitalWrite(CHANNEL_PINS[i], LOW);
    }
    activeChannel = -1;

    Serial.println("All channels reset to OFF");
    server.send(200, "application/json", "{\"success\":true,\"message\":\"All channels reset\"}");
}

void handleNotFound() {
    server.send(404, "application/json", "{\"error\":\"Not found\"}");
}
