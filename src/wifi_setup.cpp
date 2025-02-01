#include "wifi_setup.h"

extern AsyncWebServer server;
extern DNSServer dns;


void setupWiFi() {
  AsyncWiFiManager wifiManager(&server, &dns);
  wifiManager.setConfigPortalTimeout(120); // Timeout in seconds
  wifiManager.autoConnect("AutoConnectAP");
  Serial.println(WiFi.localIP());
}
