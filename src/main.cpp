#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include "config.h"

// --- Pins ---
#define SDA_PIN 8
#define SCL_PIN 9
#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

#define SLEEP_MINUTES 5
#define uS_TO_S_FACTOR 1000000ULL
#define SLEEP_TIME (SLEEP_MINUTES * 60ULL * uS_TO_S_FACTOR)

// --- BME280 + NeoPixel ---
Adafruit_BME280 bme;
Adafruit_NeoPixel pixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// --- MQTT Topics ---
const char *TEMP_STATE_TOPIC = "homeassistant/esp32s3/temperature/state";
const char *HUM_STATE_TOPIC = "homeassistant/esp32s3/humidity/state";
const char *PRES_STATE_TOPIC = "homeassistant/esp32s3/pressure/state";
const char *LED_STATE_TOPIC = "homeassistant/esp32s3/led/state";
const char *LED_CMD_TOPIC = "homeassistant/esp32s3/led/cmd";

// --- LED State ---
struct LedState {
  bool isOn = true;
  uint8_t r = 0;
  uint8_t g = 255;
  uint8_t b = 0;
  uint8_t brightness = 255;
} ledState;

bool ha_set_color = false;
bool discoveryPublished = false;

// --- Temperature color mapper ---
uint32_t temperatureToColor(float tempF)
{
  const float MIN_TEMP = 60.0;
  const float MAX_TEMP = 82.0;
  tempF = constrain(tempF, MIN_TEMP, MAX_TEMP);
  float t = (tempF - MIN_TEMP) / (MAX_TEMP - MIN_TEMP);

  uint8_t r, g, b;
  if (t < 0.5)
  {
    float x = t * 2.0;
    r = 0;
    g = 255 * x;
    b = 255 * (1.0 - x);
  }
  else
  {
    float x = (t - 0.5) * 2.0;
    r = 255 * x;
    g = 255 * (1.0 - x);
    b = 0;
  }
  return pixel.Color(r, g, b);
}

// --- Apply LED state to NeoPixel ---
void applyLedState()
{
  if (ledState.isOn)
  {
    // Apply brightness scaling
    uint8_t r = (ledState.r * ledState.brightness) / 255;
    uint8_t g = (ledState.g * ledState.brightness) / 255;
    uint8_t b = (ledState.b * ledState.brightness) / 255;
    pixel.setPixelColor(0, pixel.Color(r, g, b));
  }
  else
  {
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
  }
  pixel.show();
}

// --- Publish LED state ---
void publishLedState()
{
  StaticJsonDocument<256> doc;

  doc["state"] = ledState.isOn ? "ON" : "OFF";
  doc["brightness"] = ledState.brightness;
  doc["color_mode"] = "rgb";

  JsonObject color = doc.createNestedObject("color");
  color["r"] = ledState.r;
  color["g"] = ledState.g;
  color["b"] = ledState.b;

  char buffer[256];
  serializeJson(doc, buffer);
  mqtt.publish(LED_STATE_TOPIC, buffer, true);

  Serial.print("Published state: ");
  Serial.println(buffer);
}

// --- Wi-Fi connect ---
void connectWiFi()
{
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// --- MQTT connect ---
void connectMQTT()
{
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  
  while (!mqtt.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (mqtt.connect("ESP32S3_BME280", MQTT_USER, MQTT_PASS))
    {
      Serial.println("connected!");
      mqtt.subscribe(LED_CMD_TOPIC);
      Serial.print("Subscribed to: ");
      Serial.println(LED_CMD_TOPIC);
      discoveryPublished = false;
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(", retrying in 2s");
      delay(2000);
    }
  }
}

// --- Publish discovery JSON ---
void publishDiscovery()
{
  Serial.println("\n=== Publishing Discovery Messages ===");
  delay(500);

  // Temperature sensor
  const char *temp_config = "{\"name\":\"ESP32S3 Temperature\",\"state_topic\":\"homeassistant/esp32s3/temperature/state\",\"unit_of_measurement\":\"°F\",\"device_class\":\"temperature\",\"unique_id\":\"esp32s3_temp\",\"device\":{\"identifiers\":[\"esp32s3_bme280\"],\"name\":\"ESP32 S3 BME280\",\"model\":\"ESP32-S3\",\"manufacturer\":\"Espressif\"}}";
  bool temp_result = mqtt.publish("homeassistant/sensor/esp32s3_temp/config", temp_config, true);
  Serial.print("Temperature: ");
  Serial.println(temp_result ? "✓" : "✗");
  delay(500);

  // Humidity sensor
  const char *hum_config = "{\"name\":\"ESP32S3 Humidity\",\"state_topic\":\"homeassistant/esp32s3/humidity/state\",\"unit_of_measurement\":\"%\",\"device_class\":\"humidity\",\"unique_id\":\"esp32s3_humidity\",\"device\":{\"identifiers\":[\"esp32s3_bme280\"],\"name\":\"ESP32 S3 BME280\",\"model\":\"ESP32-S3\",\"manufacturer\":\"Espressif\"}}";
  bool hum_result = mqtt.publish("homeassistant/sensor/esp32s3_humidity/config", hum_config, true);
  Serial.print("Humidity: ");
  Serial.println(hum_result ? "✓" : "✗");
  delay(500);

  // Pressure sensor
  const char *pres_config = "{\"name\":\"ESP32S3 Pressure\",\"state_topic\":\"homeassistant/esp32s3/pressure/state\",\"unit_of_measurement\":\"hPa\",\"device_class\":\"pressure\",\"unique_id\":\"esp32s3_pressure\",\"device\":{\"identifiers\":[\"esp32s3_bme280\"],\"name\":\"ESP32 S3 BME280\",\"model\":\"ESP32-S3\",\"manufacturer\":\"Espressif\"}}";
  bool pres_result = mqtt.publish("homeassistant/sensor/esp32s3_pressure/config", pres_config, true);
  Serial.print("Pressure: ");
  Serial.println(pres_result ? "✓" : "✗");
  delay(500);

  // LED light - WITH EXPLICIT COLOR MODES
  const char *led_config = "{\"name\":\"ESP32S3 LED\",\"unique_id\":\"esp32s3_led\",\"command_topic\":\"homeassistant/esp32s3/led/cmd\",\"state_topic\":\"homeassistant/esp32s3/led/state\",\"schema\":\"json\",\"brightness\":true,\"supported_color_modes\":[\"rgb\"],\"optimistic\":false,\"device\":{\"identifiers\":[\"esp32s3_bme280\"],\"name\":\"ESP32 S3 BME280\",\"model\":\"ESP32-S3\",\"manufacturer\":\"Espressif\"}}";
  bool led_result = mqtt.publish("homeassistant/light/esp32s3_led/config", led_config, true);
  Serial.print("LED Light: ");
  Serial.println(led_result ? "✓" : "✗");
  delay(500);

  Serial.println("=== Discovery Complete ===\n");
  discoveryPublished = true;
}

// --- MQTT callback ---

// --- MQTT callback ---
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Msg [");
  Serial.print(topic);
  Serial.print("]: ");

  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);

  if (strcmp(topic, LED_CMD_TOPIC) == 0)
  {
    // Try to parse as JSON first
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (!error)
    {
      // Valid JSON - handle it
      if (doc.containsKey("state"))
      {
        const char *state = doc["state"];
        ledState.isOn = (strcmp(state, "ON") == 0);
        Serial.print("State: ");
        Serial.println(ledState.isOn ? "ON" : "OFF");
      }

      if (doc.containsKey("brightness"))
      {
        ledState.brightness = doc["brightness"];
        Serial.print("Brightness: ");
        Serial.println(ledState.brightness);
      }

      if (doc.containsKey("color"))
      {
        ledState.r = doc["color"]["r"];
        ledState.g = doc["color"]["g"];
        ledState.b = doc["color"]["b"];
        Serial.print("Color: R=");
        Serial.print(ledState.r);
        Serial.print(" G=");
        Serial.print(ledState.g);
        Serial.print(" B=");
        Serial.println(ledState.b);
        ha_set_color = true;
      }
    }
    else
    {
      // Not JSON - handle simple ON/OFF commands
      if (strcmp(message, "ON") == 0)
      {
        ledState.isOn = true;
        Serial.println("Simple command: ON");
      }
      else if (strcmp(message, "OFF") == 0)
      {
        ledState.isOn = false;
        Serial.println("Simple command: OFF");
      }
    }

    applyLedState();
    publishLedState();
  }
}

// --- Deep Sleep Helper ---
void goToSleep()
{
  Serial.println("Preparing for deep sleep...");

  // Turn off NeoPixel to save power
  pixel.clear();
  pixel.show();

  mqtt.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  delay(200);

  esp_sleep_enable_timer_wakeup(SLEEP_TIME);
  Serial.println("Entering deep sleep for " + String(SLEEP_MINUTES) + " minutes...");
  esp_deep_sleep_start();
}

// --- Setup ---
void setup()
{
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n=== ESP32 S3 BME280 Starting ===");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bme.begin(0x76))
  {
    Serial.println("ERROR: BME280 not found!");
    while (1) delay(1000);
  }
  Serial.println("BME280: OK");

  pixel.begin();
  pixel.setBrightness(255); // We'll handle brightness in software
  
  // Set initial state (green)
  ledState.isOn = true;
  ledState.r = 0;
  ledState.g = 255;
  ledState.b = 0;
  ledState.brightness = 255;
  applyLedState();
  
  Serial.println("NeoPixel: OK (GREEN)");

  connectWiFi();
  connectMQTT();
  mqtt.setCallback(mqttCallback);

  delay(1000);
  publishDiscovery();
  publishLedState();
  pixel.clear();
  pixel.show();


  Serial.println("=== Ready ===\n");
}

// --- Loop ---
void loop()
{
  if (!mqtt.connected())
  {
    connectMQTT();
    mqtt.setCallback(mqttCallback);
  }

  mqtt.loop();

  static bool published = false;

  if (!published)
  {
    float tempF = (bme.readTemperature() * 9.0 / 5.0 + 32.0) - 1.5;
    float hum = bme.readHumidity();
    float pres = bme.readPressure() / 100.0F;

    char buf[8];
    dtostrf(tempF, 1, 2, buf);
    mqtt.publish(TEMP_STATE_TOPIC, buf, true);
    dtostrf(hum, 1, 1, buf);
    mqtt.publish(HUM_STATE_TOPIC, buf, true);
    dtostrf(pres, 1, 1, buf);
    mqtt.publish(PRES_STATE_TOPIC, buf, true);

    // Optional: temperature LED for 1 second
    uint32_t color = temperatureToColor(tempF);
    pixel.setPixelColor(0, color);
    pixel.show();
    delay(1000);

    publishLedState();

    // Serial.print("Published data | Battery: ");

// #ifdef BATTERY_ADC_PIN
//     Serial.print(readBatteryVoltage());
//     Serial.println(" V");
// #else
//     Serial.println("N/A");
// #endif

    published = true;
    delay(500);
    goToSleep();
  }
}
// void loop()
// {
//   if (!mqtt.connected())
//   {
//     connectMQTT();
//     if (mqtt.connected() && !discoveryPublished)
//     {
//       delay(1000);
//       publishDiscovery();
//     }
//   }
//   mqtt.loop();

//   static unsigned long lastPublish = 0;
//   if (millis() - lastPublish >= 5000)
//   {
//     lastPublish = millis();
    
//     // Read sensors
//     float tempF = (bme.readTemperature() * 9.0 / 5.0 + 32.0) - 1.5; // Calibration offset
//     float hum = bme.readHumidity();
//     float pres = bme.readPressure() / 100.0F;

//     char buf[8];
//     dtostrf(tempF, 1, 2, buf);
//     mqtt.publish(TEMP_STATE_TOPIC, buf, true);
//     dtostrf(hum, 1, 1, buf);
//     mqtt.publish(HUM_STATE_TOPIC, buf, true);
//     dtostrf(pres, 1, 1, buf);
//     mqtt.publish(PRES_STATE_TOPIC, buf, true);

//     // Update LED color based on temperature if HA hasn't set a color
//     if (!ha_set_color && ledState.isOn)
//     {
//       uint32_t color = temperatureToColor(tempF);
//       ledState.r = (color >> 16) & 0xFF;
//       ledState.g = (color >> 8) & 0xFF;
//       ledState.b = color & 0xFF;
      
//       applyLedState();
//       publishLedState();
//     }

//     Serial.print(tempF);
//     Serial.print("°F | ");
//     Serial.print(hum);
//     Serial.print("% | ");
//     Serial.print(pres);
//     Serial.println(" hPa");
//   }
// }