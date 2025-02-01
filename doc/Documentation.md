1. PID Control Module:
We will create a separate file to handle PID-related functions and initialization. This includes setting up the PID constants, the Fermenter structure, and handling the updates for each fermenter.

2. WiFi and Web Server Module:
This module will be responsible for managing WiFi connections, the configuration portal, and handling HTTP requests through the web server.

3. Temperature Sensor Module:
Handle temperature readings and interaction with the DallasTemperature library.

4. Relay and Pump Module:
Separate the relay and pump control logic into its own module.

5. Preferences Module (storage):
Isolate code related to reading and saving setpoints and fermenter status from the preferences.

6. WebSocket Module:
Encapsulate WebSocket connection management and event handling.

