#include <ESPAsyncWebServer.h>
/*
MIT LICENCE

Copyright 2025 Michel Gauvin (michel.gauvin@gmail.com) www.mikesbrewshop.com

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "web_server.h"
#include "fermenter_control.h"
#include "relay_control.h"

// Declare the server globally (same as in your main sketch)
extern AsyncWebServer server;
extern WebSocketsServer webSocket;

// Helper function to map key to fermenter index
int getFermenterIndex(const char* key_value) {
    if (strstr(key_value, "ferm1_") != NULL) return 0;
    if (strstr(key_value, "ferm2_") != NULL) return 1;
    if (strstr(key_value, "ferm3_") != NULL) return 2;
    if (strstr(key_value, "ferm4_") != NULL) return 3;
    return -1; // Invalid fermenter index
}

// Process the JSON data for each fermenter
void processFermenterData(const char* key_value, const char* str_value, int fermIndex) {
    if (strstr(key_value, "Status") != NULL) {
        updateStatus(key_value, str_value, fermIndex);
        updateRelayStatus(fermIndex, false);
        fermenters[fermIndex].pumpStatus = false;
    } else if (strstr(key_value, "Setpoint") != NULL) {
        updateSetpoint(key_value, str_value, fermIndex);
    } else if (strstr(key_value, "ColdCrash") != NULL) {
        setPumpColdCrash(str_value, fermIndex);
    }
}

// Web server setup
void setupWebServer() {
    // Route for serving the root page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html");
    });

    // WebSocket events
    webSocket.begin();
    webSocket.onEvent(handleWebSocketEvent);

    // Route for /post to handle status, setpoint, and cold crash changes
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, data);
        if (error) {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            return;
        }

        // Process JSON data for each key-value pair
        for (JsonPair kv : doc.as<JsonObject>()) {
            const char* key_value = kv.key().c_str();
            const char* str_value = kv.value().as<const char*>();

            int fermIndex = getFermenterIndex(key_value);
            if (fermIndex != -1) {
                processFermenterData(key_value, str_value, fermIndex);
            }
        }

        request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"JSON data received\"}");
    });

    server.begin();
}

// Handle WebSocket events
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connection from ", num);
            Serial.println(ip.toString());

            // Send a welcome message to the client
            StaticJsonDocument<200> doc;
            doc["message"] = "Welcome to the WebSocket server!";
            String json;
            serializeJson(doc, json);
            webSocket.sendTXT(num, json);
        } break;
        case WStype_TEXT:
            Serial.printf("[%u] Received text: %s\n", num, payload);

            // Echo the received message back to the client
            webSocket.sendTXT(num, payload);
            break;
    }
}

// Broadcast data with websocket to client
void sendData()
{
      // Send data to websocket
  if (WiFi.status() == WL_CONNECTED) {
  for (int i = 0; i < 4; ++i)
  {
    String json = fermenters[i].toJson();
    webSocket.broadcastTXT(json);
  }
  }
  else{
    printf("No internet.\n");
  }
}