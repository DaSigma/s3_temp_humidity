# ESP32-S3 BME280 Temperature/Humidity Sensor with MQTT and NeoPixel

This project utilizes an ESP32-S3 microcontroller to read temperature, humidity, and pressure data from a BME280 sensor, publish it to an MQTT broker, and provide visual feedback via an Adafruit NeoPixel LED. It also incorporates deep sleep functionality to conserve power.

## Features

*   Reads temperature (in Fahrenheit), humidity, and atmospheric pressure from a BME280 sensor.
*   Publishes sensor data to an MQTT broker.
*   MQTT-controlled NeoPixel LED with color and brightness adjustments, and state reporting.
*   Home Assistant MQTT Discovery for automatic integration of sensors and LED.
*   Deep sleep mode to reduce power consumption, waking up periodically to take readings and publish data.
*   Temperature-to-color mapping on the NeoPixel for quick visual temperature indication.

## Hardware Requirements

*   ESP32-S3 Development Board
*   BME280 Temperature, Humidity, and Pressure Sensor
*   Adafruit NeoPixel (or compatible WS2812B RGB LED)
*   Jumper Wires and Breadboard (for prototyping)

## Software Requirements

*   [PlatformIO IDE](https://platformio.org/) (VSCode extension recommended)
*   Arduino Framework for ESP32

## Setup Instructions

1.  **Clone the repository:**
    ```bash
    git clone [your-repository-url]
    cd s3_temp_humidity
    ```

2.  **PlatformIO Setup:**
    Open the project in PlatformIO IDE. Ensure you have the ESP32 platform installed.

3.  **Configure Wi-Fi and MQTT Credentials:**
    Copy `include/config.h.example` to `include/config.h`:
    ```bash
    cp include/config.h.example include/config.h
    ```
    Open `include/config.h` and update the following placeholders with your actual network and MQTT broker details:
    *   `WIFI_SSID`: Your Wi-Fi network name.
    *   `WIFI_PASS`: Your Wi-Fi network password.
    *   `MQTT_HOST`: The IP address or hostname of your MQTT broker.
    *   `MQTT_PORT`: The port of your MQTT broker (usually 1883).
    *   `MQTT_USER`: Your MQTT username (if authentication is required).
    *   `MQTT_PASS`: Your MQTT password (if authentication is required).

    **Important:** `include/config.h` is listed in `.gitignore` and should **not** be committed to your version control system, especially if it contains sensitive credentials.

## Usage

1.  **Build and Upload:**
    Use the PlatformIO "Upload" task to compile the firmware and flash it to your ESP32-S3 board.

2.  **Serial Monitor:**
    Open the PlatformIO Serial Monitor (baud rate 115200) to view debug output, Wi-Fi connection status, MQTT communication, and sensor readings.

3.  **Home Assistant Integration:**
    If your Home Assistant instance is configured to discover MQTT devices, the ESP32-S3 sensors (Temperature, Humidity, Pressure) and the NeoPixel LED should automatically appear.

    You can control the LED via Home Assistant (e.g., toggle on/off, change brightness, set color) or by publishing to its command topic (`homeassistant/esp32s3/led/cmd`).

## MQTT Topics

*   **Sensor States:**
    *   `homeassistant/esp32s3/temperature/state`
    *   `homeassistant/esp32s3/humidity/state`
    *   `homeassistant/esp32s3/pressure/state`
*   **LED Control:**
    *   `homeassistant/esp32s3/led/state` (publishes current LED state)
    *   `homeassistant/esp32s3/led/cmd` (subscribes for LED commands, expects JSON payload)
*   **Home Assistant Discovery (published on startup):**
    *   `homeassistant/sensor/esp32s3_temp/config`
    *   `homeassistant/sensor/esp32s3_humidity/config`
    *   `homeassistant/sensor/esp32s3_pressure/config`
    *   `homeassistant/light/esp32s3_led/config`

## Deep Sleep

The device enters deep sleep for `SLEEP_MINUTES` (default 5 minutes) after taking readings and publishing data. It wakes up, reconnects to Wi-Fi/MQTT, performs its tasks, and then goes back to sleep.

## Contributing

Feel free to open issues or pull requests.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.