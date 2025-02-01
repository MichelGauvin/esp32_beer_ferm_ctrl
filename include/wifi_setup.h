#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

#include <WiFi.h>
#include <ESPAsyncWiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
extern DNSServer dns;
extern AsyncWiFiManager wifiManager;

void setupWiFi();

#endif
