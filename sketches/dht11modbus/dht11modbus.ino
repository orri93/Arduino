#include <gatldht.h>
#include <gatltick.h>

#define DHT11_PIN              2

#define DHT11_INO_BUFFER_SIZE 64

::gos::atl::dht::Values<GATL_DHT_DEFAULT_RAW_TYPE> raw;
::gos::atl::dht::Values<GATL_DHT_DEFAULT_VALUE_REAL_TYPE> real;
::gos::atl::dht::Values<GATL_DHT_DEFAULT_VALUE_INT_TYPE> integer;

GATL_DHT_DEFAULT_RESAULT_TYPE dhtres;

unsigned long tick, next = 0;

char buffer[DHT11_INO_BUFFER_SIZE];

char buffer_rh[DHT11_INO_BUFFER_SIZE];
char buffer_rt[DHT11_INO_BUFFER_SIZE];

void setup() {
  Serial.begin(9600);
  Serial.write("t,dhtr,rh,rt,ih,it,rh,rt\n");
}

void loop() {
  tick = millis();
  if (::gos::atl::tick::is::next<unsigned long>(next, tick, GATL_DHT_INTERVAL_DHT11)) {
    dhtres = ::gos::atl::dht::read(raw, DHT11_PIN, GATL_DHT_DELAY_DHT11);
    if (dhtres == GATL_DHT_SUCCESS) {
      ::gos::atl::dht::convert::real::dht11(real, raw);
      ::gos::atl::dht::convert::integer::dht11(real, integer);
    }
    dtostrf(real.humidity, 3, 1, buffer_rh);
    dtostrf(real.temperature, 3, 1, buffer_rt);
    snprintf(buffer, DHT11_INO_BUFFER_SIZE, "%lu,%d,%d,%d,%d,%d,%s,%s\n",
      tick,
      dhtres,
      raw.humidity,
      raw.temperature,
      integer.humidity,
      integer.temperature,
      buffer_rh,
      buffer_rt);
    Serial.write(buffer);
  }
}
