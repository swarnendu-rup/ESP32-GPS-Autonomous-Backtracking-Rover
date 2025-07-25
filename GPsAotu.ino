// =======================================================================
//   AUTONOMOUS GPS BACKTRACKING BOT - ESP32 FIRMWARE
// =======================================================================

// --- Core Libraries ---
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ArduinoJson.h>
#include <vector>

// --- Sensor Libraries ---
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// --- WiFi Credentials ---
const char* ssid = "MySpyCar"; // Your WiFi SSID
const char* password = "123456789"; // Your WiFi Password

// --- GPS Configuration ---
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(2);

// --- Motor Driver (L298N) Pin Configuration ---
#define ENA 25 // Left motor speed
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 33 // Right motor speed
const int motorSpeed = 150; // Speed from 0-255

// --- Compass Sensor ---
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// --- Navigation & State Management ---
enum BotState { IDLE, EXPLORING, BACKTRACKING, EMERGENCY_STOP };
BotState currentState = IDLE;

struct GPSCoordinate {
  double lat;
  double lon;
};

std::vector<GPSCoordinate> loggedPath;
int backtrackingWaypointIndex = -1;
const double WAYPOINT_RADIUS_METERS = 2.0; // How close to get to a waypoint
const double LOG_DISTANCE_METERS = 3.0;   // How far to travel before logging a new point

// --- Web Server & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =======================================================================
// MOTOR CONTROL FUNCTIONS
// =======================================================================
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH); // Left wheels forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Right wheels backward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW); // Left wheels backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); // Right wheels forward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =======================================================================
// NAVIGATION LOGIC
// =======================================================================
void navigateToWaypoint() {
    if (backtrackingWaypointIndex < 0 || !gps.location.isValid()) {
        stopMotors();
        currentState = IDLE;
        return;
    }

    GPSCoordinate target = loggedPath[backtrackingWaypointIndex];
    double distanceToTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), target.lat, target.lon);

    // Check if we have arrived at the waypoint
    if (distanceToTarget < WAYPOINT_RADIUS_METERS) {
        backtrackingWaypointIndex--; // Target the next point in the path
        if (backtrackingWaypointIndex < 0) { // We have arrived home
            currentState = IDLE;
            stopMotors();
            return;
        }
    }

    // --- Navigation Calculation ---
    double targetBearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target.lat, target.lon);

    sensors_event_t event;
    mag.getEvent(&event);
    float currentHeading = atan2(event.magnetic.y, event.magnetic.x) * (180 / PI);
    if (currentHeading < 0) currentHeading += 360;

    double headingError = targetBearing - currentHeading;
    // Normalize the error to be between -180 and 180
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // --- Motor Commands based on Heading Error ---
    if (abs(headingError) < 15) { // If we are pointing in the right direction
        moveForward();
    } else if (headingError > 0) {
        turnRight();
    } else {
        turnLeft();
    }
}


// =======================================================================
// WEBSOCKET AND DATA HANDLING
// =======================================================================
void broadcastData() {
    String jsonString;
    StaticJsonDocument<1024> doc; // Increased size to handle path data

    doc["lat"] = gps.location.lat();
    doc["lon"] = gps.location.lng();
    doc["sats"] = gps.satellites.value();
    doc["speed"] = gps.speed.kmph();
    doc["valid"] = gps.location.isValid();
    
    // Convert enum state to string
    switch(currentState) {
        case IDLE: doc["state"] = "IDLE"; break;
        case EXPLORING: doc["state"] = "EXPLORING"; break;
        case BACKTRACKING: doc["state"] = "BACKTRACKING"; break;
        case EMERGENCY_STOP: doc["state"] = "STOPPED"; break;
    }

    doc["points"] = loggedPath.size();

    // Send the full path only to a newly connected client
    if (loggedPath.size() > 0) {
        JsonArray path = doc.createNestedArray("path");
        for(const auto& p : loggedPath) {
            JsonObject point = path.createNestedObject();
            point["lat"] = p.lat;
            point["lon"] = p.lon;
        }
    }

    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}


void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client #%u connected\n", client->id());
        broadcastData(); // Send initial full data dump
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg; 
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            StaticJsonDocument<64> doc;
            if (deserializeJson(doc, (char*)data) == DeserializationError::Ok) {
                const char* command = doc["command"];
                if (strcmp(command, "explore") == 0) {
                    currentState = EXPLORING;
                } else if (strcmp(command, "return") == 0) {
                    if (!loggedPath.empty()) {
                        backtrackingWaypointIndex = loggedPath.size() - 1; // Start from the last point
                        currentState = BACKTRACKING;
                    }
                } else if (strcmp(command, "stop") == 0) {
                    currentState = EMERGENCY_STOP;
                }
            }
        }
    }
}

// =======================================================================
// WEBPAGE HTML CONTENT
// =======================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Bot Command & Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <style>
        body, html { margin: 0; padding: 0; height: 100%; width: 100%; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; overflow: hidden; }
        .grid-container { display: grid; grid-template-columns: 300px 1fr; grid-template-rows: 100vh; height: 100%; }
        #map { height: 100%; width: 100%; background-color: #333; }
        .panel { background-color: #f0f2f5; padding: 15px; display: flex; flex-direction: column; }
        .panel h2 { margin: 0 0 15px 0; text-align: center; }
        .controls button { display: block; width: 100%; padding: 15px; margin-bottom: 10px; font-size: 16px; border: none; border-radius: 5px; color: white; cursor: pointer; }
        #btn-explore { background-color: #28a745; }
        #btn-return { background-color: #007bff; }
        #btn-stop { background-color: #dc3545; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 20px; }
        .info-box { background: #fff; padding: 10px; border-radius: 5px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
        .info-box .label { font-size: 12px; color: #6c757d; }
        .info-box .value { font-size: 18px; font-weight: bold; text-align: center; }
        #state { color: #fd7e14; }
    </style>
</head>
<body>
    <div class="grid-container">
        <div class="panel">
            <h2>Bot C&C</h2>
            <div class="controls">
                <button id="btn-explore" onclick="sendCommand('explore')">Start Exploring</button>
                <button id="btn-return" onclick="sendCommand('return')">Return to Home</button>
                <button id="btn-stop" onclick="sendCommand('stop')">EMERGENCY STOP</button>
            </div>
            <div class="status-grid">
                <div class="info-box"><div class="label">STATE</div><div class="value" id="state">CONNECTING...</div></div>
                <div class="info-box"><div class="label">GPS</div><div class="value" id="gps-status">NO FIX</div></div>
                <div class="info-box"><div class="label">SATELLITES</div><div class="value" id="sats">--</div></div>
                <div class="info-box"><div class="label">SPEED (KM/H)</div><div class="value" id="speed">--</div></div>
                <div class="info-box"><div class="label">PATH POINTS</div><div class="value" id="points">--</div></div>
                <div class="info-box"><div class="label">HOME</div><div class="value" id="home-coords">N/A</div></div>
            </div>
            <div class="info-box" style="margin-top: 10px;">
                <div class="label">CURRENT LOCATION</div>
                <div class="value" id="current-coords">N/A</div>
            </div>
        </div>
        <div id="map"></div>
    </div>
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
        const map = L.map('map').setView([20, 0], 2);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

        let botMarker = null, homeMarker = null;
        let pathPolyline = L.polyline([], {color: '#007bff', weight: 5}).addTo(map);

        function sendCommand(cmd) {
            ws.send(JSON.stringify({ "command": cmd }));
        }

        const ws = new WebSocket(`ws://${location.host}/ws`);
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);

            // Update text displays
            document.getElementById('state').textContent = data.state || 'N/A';
            document.getElementById('gps-status').textContent = data.valid ? 'VALID' : 'NO FIX';
            document.getElementById('sats').textContent = data.sats || '--';
            document.getElementById('speed').textContent = data.speed || '--';
            document.getElementById('points').textContent = data.points || '0';
            
            if (data.valid) {
                const latLng = [data.lat, data.lon];
                document.getElementById('current-coords').textContent = `${data.lat.toFixed(5)}, ${data.lon.toFixed(5)}`;
                
                if (!botMarker) {
                    botMarker = L.marker(latLng).addTo(map);
                    map.setView(latLng, 18);
                } else {
                    botMarker.setLatLng(latLng);
                    map.panTo(latLng);
                }
            }
            
            // Update the entire path if provided
            if (data.path && data.path.length > 0) {
                const pathCoords = data.path.map(p => [p.lat, p.lon]);
                pathPolyline.setLatLngs(pathCoords);
                
                const homeCoords = data.path[0];
                document.getElementById('home-coords').textContent = `${homeCoords.lat.toFixed(5)}, ${homeCoords.lon.toFixed(5)}`;
                if(!homeMarker) {
                   homeMarker = L.marker([homeCoords.lat, homeCoords.lon], {
                       icon: L.icon({ iconUrl: 'https://cdn.rawgit.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png', shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png', iconSize: [25, 41], iconAnchor: [12, 41] })
                   }).addTo(map).bindPopup("Home").openPopup();
                }
            }
        };
    </script>
</body>
</html>
)rawliteral";


// =======================================================================
// SETUP AND LOOP
// =======================================================================

void setup() {
    Serial.begin(115200);
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Wire.begin();

    // Initialize Motor Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stopMotors();

    // Initialize Compass
    if(!mag.begin()){
      Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
      while(1);
    }

    // Initialize WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

    // Initialize Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    ElegantOTA.begin(&server);
    server.begin();
}

unsigned long lastBroadcastTime = 0;

void loop() {
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    // Main State Machine Logic
    switch(currentState) {
        case EXPLORING:
            moveForward();
            if (gps.location.isValid()) {
                if (loggedPath.empty()) {
                    // Log the very first point (home)
                    loggedPath.push_back({gps.location.lat(), gps.location.lng()});
                } else {
                    double distFromLast = TinyGPSPlus::distanceBetween(
                        gps.location.lat(), gps.location.lng(),
                        loggedPath.back().lat, loggedPath.back().lon
                    );
                    if (distFromLast > LOG_DISTANCE_METERS) {
                        loggedPath.push_back({gps.location.lat(), gps.location.lng()});
                    }
                }
            }
            break;

        case BACKTRACKING:
            navigateToWaypoint();
            break;
            
        case IDLE:
        case EMERGENCY_STOP:
            stopMotors();
            break;
    }

    // Broadcast data periodically
    if (millis() - lastBroadcastTime > 1000) {
        lastBroadcastTime = millis();
        broadcastData();
    }
    ElegantOTA.loop();
}