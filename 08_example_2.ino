// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() { 
  float distance;

  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // Check distance and control LED brightness
  if (distance < _DIST_MIN) {
      analogWrite(PIN_LED, 255); // LED off at minimum distance
  } else if (distance > _DIST_MAX) {
      analogWrite(PIN_LED, 255); // LED off at maximum distance
  } else {
      int brightness = calculateLEDbrightness(distance);
      analogWrite(PIN_LED, brightness); // adjust LED brightness
  }

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");
  
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

// calculate LED brightness based on distance
int calculateLEDbrightness(float distance) {
  float brightness;

  if (distance <= 150.0) {
      // between 100mm and 150mm (bright)
      brightness = map(distance, 100.0, 150.0, 255, 128); // scale brightness (200mm brightest)
  } else if (distance <= 250.0) {
      // between 150mm and 250mm (half bright)
      brightness = map(distance, 150.0, 250.0, 128, 128); // 50% duty cycle brightness
  } else {
      // between 250mm and 300mm (dim)
      brightness = map(distance, 250.0, 300.0, 128, 255); // scale brightness (300mm dimmest)
  }

  return (int)brightness;
}
