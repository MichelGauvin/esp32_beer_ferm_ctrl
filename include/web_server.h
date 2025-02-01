#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

extern AsyncWebServer server;
extern WebSocketsServer webSocket;

void setupWebServer();
void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendData();

#endif
