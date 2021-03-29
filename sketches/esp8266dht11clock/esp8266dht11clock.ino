#include <Wire.h>
#include <SSD1306Wire.h>

#include <NTPClient.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>

#include <WiFiUdp.h>

#include <PubSubClient.h>

#include <dht.h>

#include <gatltick.h>

#define DHT11_PIN                   D0
#define PUSH_BUTTON_PIN             D1
#define DISPLAY_SDA_PIN             D7
#define DISPLAY_SCL_PIN             D5
#define DISPLAY_ADDRESS           0x3c

#define MQTT_PORT                "1883"
#define MQTT_SERVER           "0.0.0.0"
#define MQTT_CLIENT_ID   "dht11esp8266"
#define MQTT_TOPIC            "iot/dht"

#define MQTT_HOST_COUNT              1

#define BAUD_RATE               115200

#define DELAY_SETUP_END            200

#define INTERVAL_DHT11            2500
#define INTERVAL_CLOCK             100
#define INTERVAL_CONNECT_MQTT      100
#define INTERVAL_CONNECT_WIFI      250

#define BUFFER_SIZE                 64
#define REAL_BUFFER_SIZE             8

#define REAL_SIZE                    6
#define REAL_PRECISION               2

#define JSON_FORMAT     "{\"t\": %lu, \"dht11\": %d, \"RH\": %s, \"T\": %s }"
#define DISPLAY_FORMAT  "%s °C %s %%"

namespace gat = gos::atl::tick;

WiFiManager wifiManager;

WiFiManagerParameter mqttport("port", "mqtt port", MQTT_PORT, sizeof(MQTT_PORT));
WiFiManagerParameter mqttserver("server", "mqtt server", MQTT_SERVER, sizeof(MQTT_SERVER));
WiFiManagerParameter mqttclientid("client", "mqtt client id", MQTT_CLIENT_ID, sizeof(MQTT_CLIENT_ID));
WiFiManagerParameter mqtttopic("topic", "mqtt topic", MQTT_TOPIC, sizeof(MQTT_TOPIC));

SSD1306Wire display(DISPLAY_ADDRESS, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN, GEOMETRY_128_32);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String formattedTime;

dht dhtinst;
int dhtres;

char buffer[BUFFER_SIZE];
char bufhum[REAL_BUFFER_SIZE];
char buftem[REAL_BUFFER_SIZE];

unsigned long tick, nextdht11 = 0, nextclock = 0;

bool isdisplay;

void setup() {
  pinMode(PUSH_BUTTON_PIN, INPUT);
  // pinMode(DHT11_PIN, INPUT);

  Serial.begin(BAUD_RATE);
  Serial.println();
  Serial.println();

  wifiManager.addParameter(&mqttport);
  wifiManager.addParameter(&mqttserver);
  wifiManager.addParameter(&mqttclientid);
  wifiManager.addParameter(&mqtttopic);

  Serial.println("Initialize the Display");
  display.init();

  Serial.println("Flip the Display screen and set font");
  display.flipScreenVertically();

  Serial.println("Clear the Display and write a message");
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  if (digitalRead(PUSH_BUTTON_PIN)) {
    Serial.println("WiFi Manager reset settings before auto connect");
    wifiManager.resetSettings();
    String cpssid = wifiManager.getConfigPortalSSID();
    display.drawString(0, 0, cpssid);
  } else {
    Serial.println("WiFi Manager auto connect");
    display.drawString(0, 0, "Auto Connect");
  }
  display.display();

  if (wifiManager.autoConnect()) {
    Serial.println("WiFi Manager Auto Connect returned true");
    String ipstr = WiFi.localIP().toString();
    display.clear();
    display.drawString(0, 0, ipstr);
    display.display();

    timeClient.begin();
  } else {
    Serial.println("WiFi Manager Auto Connect returned false");
    display.clear();
    display.drawString(0, 0, "Timeout");
    display.display();
    delay(3000);
    ESP.reset();
    delay(5000);
  }
}

void loop() {
  isdisplay = false;
  tick = millis();

  if (gat::is::next<>(nextdht11, tick, INTERVAL_DHT11)) {
    yield();
    dhtres = dhtinst.read11(DHT11_PIN);
    dtostrf(dhtinst.humidity, REAL_SIZE, REAL_PRECISION, bufhum);
    dtostrf(dhtinst.temperature, REAL_SIZE, REAL_PRECISION, buftem);
    switch(dhtres) {
      case DHTLIB_OK:
        snprintf(buffer, BUFFER_SIZE, DISPLAY_FORMAT, buftem, bufhum);
        break;
      case DHTLIB_ERROR_CHECKSUM:
        Serial.println("DHT Checksum error");
        break;
      case DHTLIB_ERROR_TIMEOUT:
        Serial.println("DHT Timeout");
        break;
      default:
        Serial.print("DHT Unknown error code ");
        Serial.println(dhtres);
        break;
    }
    isdisplay = true;
  }

  if (gat::is::next<>(nextclock, tick, INTERVAL_CLOCK) && WiFi.isConnected()) {
    timeClient.update();
    formattedTime = timeClient.getFormattedTime();
    isdisplay = true;
  }

  if (isdisplay) {
    display.clear();
    display.drawString(0, 0, formattedTime);
    display.drawString(0, 16, buffer);
    display.display();
  }
}
