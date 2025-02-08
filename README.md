
This code provides a **network-enabled, real-time, multi-fermenter temperature control system** with **remote monitoring capabilities**. The use of PID control ensures **precise temperature stability**, making it ideal for fermentation management.

This code implements a **fermentation temperature control system** for up to **four fermenters** using an **ESP32 (or similar ESP-based microcontroller)**. The system is designed to monitor and regulate temperature using **PID controllers**, **DS18B20 temperature sensors**, and **relays** to control heating or cooling elements. Here's a high-level breakdown of its functionality:

### **Main Features**
1. **Temperature Sensing**  
   - Uses **DS18B20 temperature sensors** on a **OneWire bus** to measure temperature.  
   - Each fermenter has a pre-registered unique **sensor address**.

2. **PID Control**  
   - Implements **Proportional-Integral-Derivative (PID) control loops** for precise temperature regulation.  
   - Uses different PID tuning parameters (`Kp`, `Ki`, `Kd`) for **small and large fermenters**.  
   - The control logic is **inverted**, meaning relay activation logic may be reversed.

3. **WiFi & Web Server**  
   - Connects to WiFi via `setupWiFi()`.  
   - Hosts an **async web server** (`ESPAsyncWebServer`) on port **80** for remote monitoring and control.  
   - Supports **WebSockets** (port **3333**) for real-time communication with clients.

4. **Persistent Storage**  
   - Uses **LittleFS** for storing and retrieving configuration data.  
   - Saves fermenter **setpoints** and **status** for persistence across reboots.

5. **Relay Control**  
   - Interfaces with relays to **turn heating/cooling devices on/off** based on PID output.  
   - Relays are initialized in `setupRelays()`.

6. **Event Loop & Timing**  
   - The `loop()` function:  
     - Reads temperatures from sensors every **1 second**.  
     - Runs the **PID control loop** to adjust fermenter temperatures.  
     - Sends updated data to connected clients.  
     - Handles WebSocket events.

### **TODO List**
- **Synchronize time with NTP** for accurate logging and scheduling.  
- **WiFi reconfiguration mode** (pressing a button during reboot to enter setup mode).  
- **Clarify inverted relay logic** in documentation.  
- **Make PID settings adjustable** via the web interface.  
- **Dynamically detect and assign DS18B20 sensors** instead of hardcoding addresses.

# Configuration

Il faut mettre la configuration suivante dans le fichier suivant qui est utilisé par le conteneur nginx:

fichier: /srv/docker/nginx/etc/nginx.conf

Mettre la configuration suivante à la fin de l'élément http {}:

        server {
            listen 443 ssl;
            server_name nodemcu.mikesbrewshop.com;

            ssl_certificate /etc/nginx/certificate/fullchain1.pem;
            ssl_certificate_key /etc/nginx/certificate/privkey1.pem;

            location / {
                proxy_pass http://10.0.0.143;
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }

            location /ws {
                proxy_pass http://10.0.0.143:3333;  # WebSocket server port
                proxy_http_version 1.1;
                proxy_set_header Upgrade $http_upgrade;
                proxy_set_header Connection "upgrade";
                proxy_set_header Host $host;
            }
        }
