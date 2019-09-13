/*
 *
 * https://create.arduino.cc/projecthub/Johan_Ha/from-ky-039-to-heart-rate-0abfca
 *
 * From KY-039 To Heart Rate © CC BY-NC
 * Get your heart rate, not just an IR sensor reading.
 *
 * In the set of 37 sensors for Arduino, there's a heartbeat sensor.
 * The name promises too much. People tend to think it provides
 * a digital number through I2C or something similar, a number which is
 * the heartbeat rate. What the sensor provides is just an "analog" value
 * from 0 to 1023, telling how much infra red light the light sensor receives,
 * or actually how much something is shading the light sensor.
 * The higher value, the less infra red light.
 *
 * In short: place your finger between the IR led and the light transistor of
 * the sensor. Your heartbeat dilates the blood vessels in your finger,
 * which will filter the IR. This creates a pulsating signal.
 * 
 * In this project I describe how this signal is converted into
 * a heartbeat rate like 66 BPM (beats per minute).
 *
 */

#define samp_siz 4
#define rise_threshold 5
// Pulse Monitor Test Script
int sensorPin = A2;
void setup() {
   Serial.begin(9600);
}
void loop ()
{
   float reads[samp_siz], sum;
   long int now, ptr;
   float last, reader, start;
   float first, second, third, before, print_value;
   bool rising;
   int rise_count;
   int n;
   long int last_beat;
   for (int i = 0; i < samp_siz; i++)
     reads[i] = 0;
   sum = 0;
   ptr = 0;
   while(1)
   {
     // calculate an average of the sensor
     // during a 20 ms period (this will eliminate
     // the 50 Hz noise caused by electric light
     n = 0;
     start = millis();
     reader = 0.;
     do
     {
       reader += analogRead (sensorPin);
       n++;
       now = millis();
     }
     while (now < start + 20);  
     reader /= n;  // we got an average
     // Add the newest measurement to an array
     // and subtract the oldest measurement from the array
     // to maintain a sum of last measurements
     sum -= reads[ptr];
     sum += reader;
     reads[ptr] = reader;
     last = sum / samp_siz;
     //Serial.println(last, DEC);
     // now last holds the average of the values in the array
     // check for a rising curve (= a heart beat)
     if (last > before)
     {
       rise_count++;
       if (!rising && rise_count > rise_threshold)
       {
         // Ok, we have detected a rising curve, which implies a heartbeat.
         // Record the time since last beat, keep track of the two previous
         // times (first, second, third) to get a weighed average.
         // The rising flag prevents us from detecting the same rise 
         // more than once.
         rising = true;
         first = millis() - last_beat;
         last_beat = millis();
         // Calculate the weighed average of heartbeat rate
         // according to the three last beats
         print_value = 60000. / (0.4 * first + 0.3 * second + 0.3 * third);
         Serial.println(print_value);
         
         third = second;
         second = first;
       }
     }
     else
     {
       // Ok, the curve is falling
       rising = false;
       rise_count = 0;
     }
     before = last;
     ptr++;
     ptr %= samp_siz;
   }
} 
