# ESP32-GPS-Autonomous-Backtracking-Rover
An autonomous ESP32 rover that uses a GPS module and an HMC5883L compass to explore and log its path. It can automatically backtrack to its starting point by navigating the logged waypoints. The rover is controlled and monitored via a WiFi web interface that displays its live position and path on a map. Connections: L298N, GPS, HMC5883L.

The Autonomous GPS Backtracking Bot is a sophisticated project built around the ESP32 microcontroller, designed to navigate autonomously using GPS and a compass, log its path, and backtrack to its starting point. Below, I detail the connection steps for the hardware, the features of the bot, and the functionality of its web interface, ensuring a comprehensive explanation.

Hardware Connection Steps
To assemble the bot, follow these steps to connect the components correctly:

ESP32 Microcontroller Setup:
Use an ESP32 development board (e.g., ESP32 DevKitC) as the central controller.
Connect it to a stable 5V power source (e.g., a battery pack or USB power bank) via the VIN pin or USB port.
Ensure proper grounding by connecting all GND pins to a common ground rail.
L298N Motor Driver Connections:
Connect the motor driver to control two DC motors for differential drive.
Wire IN1 (GPIO 26) and IN2 (GPIO 27) to control the left motor’s direction.
Wire IN3 (GPIO 14) and IN4 (GPIO 12) for the right motor’s direction.
Connect ENA (GPIO 25) and ENB (GPIO 33) to PWM-capable pins for speed control.
Attach the motor driver’s power input to a 6-12V battery (depending on motor ratings) and connect its GND to the common ground.
Connect the left and right motors to the driver’s output terminals.
GPS Module (NEO-6M or similar):
Connect the GPS module’s TX pin to ESP32’s RX pin (GPIO 16) and RX pin to TX pin (GPIO 17).
Power the GPS module with 3.3V or 5V (check module specs) from the ESP32’s corresponding power pin.
Connect the GPS module’s GND to the common ground.
Ensure the GPS antenna has a clear view of the sky for satellite acquisition.
HMC5883L Compass Module:
Connect the compass to the ESP32 via I2C: SDA to GPIO 21 and SCL to GPIO 22 (default I2C pins).
Power the module with 3.3V from the ESP32 and connect GND to the common ground.
Verify the module’s address (default: 0x1E) to ensure proper communication.
Power Management:
Use a step-down buck converter if powering the ESP32 and GPS from a higher-voltage battery (e.g., 7.4V LiPo).
Ensure the motor driver and ESP32 share a common ground to prevent voltage mismatches.
Add a power switch for convenience and a fuse for safety.
Chassis and Mechanical Assembly:
Mount the motors, wheels, and caster wheel on a suitable chassis (e.g., acrylic or aluminum).
Secure the ESP32, motor driver, GPS module, and compass to the chassis, ensuring the GPS antenna is elevated and the compass is away from magnetic interference.
Features of the Bot:
The bot is a feature-rich autonomous vehicle with the following capabilities:

Autonomous Navigation: In EXPLORING mode, the bot moves forward, logging GPS coordinates every 3 meters to create a path. It uses the HMC5883L compass to maintain orientation.
Backtracking: In BACKTRACKING mode, the bot autonomously navigates back to its starting point by following the logged GPS waypoints in reverse order, using a PID-like heading correction based on compass data.
GPS Path Logging: The bot stores GPS coordinates (latitude, longitude) in a vector, only logging new points when the distance from the last point exceeds 3 meters, optimizing memory usage.
Differential Drive: Two DC motors controlled via the L298N driver allow forward movement, left/right turns, and stopping, with PWM speed control set to a fixed value (150/255).
WiFi and Web Interface: The ESP32 hosts a WiFi access point (SSID: "MySpyCar", Password: "123456789") and an AsyncWebServer on port 80, enabling remote control and monitoring.
WebSocket Communication: Real-time data (GPS location, speed, satellite count, state, and path) is broadcast every second to connected clients via WebSocket.
OTA Updates: ElegantOTA enables wireless firmware updates through the web interface, simplifying maintenance.
Emergency Stop: An EMERGENCY_STOP state halts all motor activity for safety, triggered via the web interface.
Webpage Functionality
The web interface, served at the ESP32’s IP address, is a responsive, user-friendly control panel built with HTML, CSS, and JavaScript, utilizing Leaflet.js for mapping. Key features include:

Layout: The page uses a grid layout with a 300px-wide control panel on the left and a full-screen map on the right.
Control Buttons:
Start Exploring: Initiates EXPLORING mode, commanding the bot to move forward and log its path.
Return to Home: Triggers BACKTRACKING mode, directing the bot to follow its logged path back to the starting point.
Emergency Stop: Halts all movement, setting the state to EMERGENCY_STOP.
Status Display: A grid of info boxes shows real-time data:
State: Displays the bot’s current state (IDLE, EXPLORING, BACKTRACKING, STOPPED).
GPS Status: Indicates if the GPS has a valid fix (“VALID” or “NO FIX”).
Satellites: Shows the number of satellites in view.
Speed: Displays speed in km/h.
Path Points: Shows the number of logged waypoints.
Home Coordinates: Displays the starting point’s latitude and longitude.
Current Location: Shows the bot’s current GPS coordinates.
Map Visualization: The Leaflet map displays:
A blue polyline tracing the bot’s logged path.
A marker for the bot’s current location, updated in real-time.
A red home marker at the starting point, with a popup label.
The map pans to follow the bot and zooms to level 18 when a valid GPS fix is acquired.
WebSocket Updates: The interface receives JSON data every second, updating the map and status displays seamlessly.
Conclusion
This autonomous GPS backtracking bot combines robust hardware integration with a feature-rich firmware and an intuitive web interface. The connection steps ensure reliable operation, while the bot’s navigation, logging, and backtracking capabilities make it ideal for applications like exploration or surveillance. The web interface provides real-time control and visualization, enhanced by OTA updates for easy maintenance. This project showcases the ESP32’s versatility in robotics and IoT applications
