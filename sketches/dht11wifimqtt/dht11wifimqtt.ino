#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <PubSubClient.h>

#include <dht.h>

#include <gatltick.h>

#define DHT11_PIN                   D1

#define MQTT_CLIENT_ID   "dht11esp8266"
#define MQTT_TOPIC            "iot/dht"
#define MQTT_PORT                 1883

#define MQTT_HOST_COUNT              1

#define BAUD_RATE               115200

#define DELAY_SETUP_END            200

#define INTERVAL_MAIN             2000
#define INTERVAL_CONNECT_MQTT      100
#define INTERVAL_CONNECT_WIFI      250

#define BUFFER_SIZE                 64
#define REAL_BUFFER_SIZE             8

#define REAL_SIZE                    6
#define REAL_PRECISION               2

#define JSON_FORMAT "{\"t\": %lu, \"dht11\": %d, \"RH\": %s, \"T\": %s }"

typedef const char* MqttHost;

dht dhtinst;

ESP8266WiFiMulti multiwifi;
WiFiClient wificlient;

PubSubClient mqttclient(wificlient);

MqttHost mqtthostlist[MQTT_HOST_COUNT] = {
  "192.168.10.168"
};

MqttHost mqtthostavailable = nullptr;

String textval;

unsigned long tick, next = 0, lastmqttconnected = 0;

char buffer[BUFFER_SIZE];
char bufhum[REAL_BUFFER_SIZE];
char buftem[REAL_BUFFER_SIZE];

byte mqtthostindex = 0, mqtthostat = 0;

int dhtres;

void startwifi();
void mqttconnect();

void mqttreceive(char* topic, byte* payload, unsigned int length) {
  /* Do nothing */
}

void setup() {
  multiwifi.addAP("Altibox105208", "Yqe6xM9C");

  Serial.begin(BAUD_RATE);
  Serial.println("DHT11 MQTT Client");

  startwifi();

  mqttclient.setServer(mqtthostlist[mqtthostindex], MQTT_PORT);
  mqttclient.setCallback(mqttreceive);

  delay(DELAY_SETUP_END);

  lastmqttconnected = millis();
}

void loop() {
  tick = millis();

  mqttclient.loop();

  yield();

  if(mqttclient.connected()) {
    lastmqttconnected = tick;

    if (::gos::atl::tick::is::next<unsigned long>(next, tick, INTERVAL_MAIN)) {
      dhtres = dhtinst.read11(DHT11_PIN);

      dtostrf(dhtinst.humidity, REAL_SIZE, REAL_PRECISION, bufhum);
      dtostrf(dhtinst.temperature, REAL_SIZE, REAL_PRECISION, buftem);

      snprintf(buffer, BUFFER_SIZE, JSON_FORMAT,
        tick,
        dhtres,
        bufhum,
        buftem);

      mqttclient.publish(MQTT_TOPIC, buffer);
      
      yield();

      switch(dhtres) {
      case DHTLIB_OK:
        snprintf(buffer, BUFFER_SIZE, "%s,%s\n", bufhum, buftem);
        Serial.write(buffer);
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
    }
  } else {
    mqttconnect();
  }
}

void startwifi() {
  Serial.print("Connecting to WiFi.");

  // Wait for the Wi-Fi to connect
  while (multiwifi.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(INTERVAL_CONNECT_WIFI);
  }

  textval = WiFi.SSID();
  Serial.print("  Connected to ");
  Serial.println(textval);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void mqttconnect() {
  Serial.println("Starting MQTT attempt");
  
  if(mqtthostavailable == nullptr) {
    mqtthostindex = mqtthostat++;
    mqttclient.setServer(mqtthostlist[mqtthostindex], MQTT_PORT);
    if(mqtthostat >= MQTT_HOST_COUNT) {
      mqtthostat = 0;
    }
  }

  if (mqttclient.connect(MQTT_CLIENT_ID)) {
    if(mqtthostavailable == nullptr) {
      mqtthostavailable = mqtthostlist[mqtthostindex];
    }
    Serial.print("Connected to MQTT broker ");
    Serial.println(mqtthostavailable);
  } else {
    yield();
    Serial.print("Failed to connect to MQTT broker ");
    Serial.print(mqtthostlist[mqtthostindex]);
    Serial.print(" through port ");
    Serial.println(MQTT_PORT);
    yield();
  }
}
