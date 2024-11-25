#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9
#define PIN_SERVO 10
#define PIN_IR    A0

// Event interval parameters
#define _INTERVAL_DIST    20   // Distance sensor interval (unit: ms) -> 더 자주 측정
#define _INTERVAL_SERVO   20   // Servo interval (unit: ms)
#define _INTERVAL_SERIAL  500  // Serial interval (unit: ms)

// EMA filter configuration for the IR distance sensor
#define _EMA_ALPHA 0.8   // EMA weight of new sample (range: 0 to 1)

// Servo adjustment - Set _DUTY_MAX, _NEU, _MIN with your own numbers
#define _DUTY_MAX 2500   // Maximum duty for a full 180 degree rotation (adjusted)
#define _DUTY_NEU 1500   // Neutral point (Center)
#define _DUTY_MIN 500    // Minimum duty for a full 180 degree rotation (adjusted)

#define _BANGBANG_RANGE 300   // Reduced duty up and down for bangbang control to avoid overshooting

#define _DIST_TARGET 175      // Target distance (unit: mm)
#define _DIST_TOLERANCE 5     // Tolerance range for the target distance

// Global variables
Servo myservo;

float dist_ema;  // Filtered distance (unit: mm)

int duty_change_per_interval; // maximum duty difference per interval
int duty_target;    // Target duty
int duty_current;   // Current duty

unsigned long last_sampling_time_dist;   // unit: msec
unsigned long last_sampling_time_servo;  // unit: msec
unsigned long last_sampling_time_serial; // unit: msec

bool event_dist, event_servo, event_serial; // Event flags

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  duty_target = duty_current = _DUTY_NEU;
  myservo.writeMicroseconds(duty_current);

  Serial.begin(115200);

  // Calculate the duty_change_per_interval based on the servo speed and angle difference
  duty_change_per_interval = 
    (float)(_DUTY_MAX - _DUTY_MIN) * (_INTERVAL_SERVO / 1000.0);
}

void loop() {
  unsigned long time_curr = millis();

  // Check event timing
  if (time_curr >= (last_sampling_time_dist + _INTERVAL_DIST)) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= (last_sampling_time_servo + _INTERVAL_SERVO)) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= (last_sampling_time_serial + _INTERVAL_SERIAL)) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  // Distance sensor event
  if (event_dist) {
    float dist_filtered;
    event_dist = false;

    // Get a distance reading
    dist_filtered = volt_to_distance(ir_sensor_filtered(15, 0.5, 0));
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

    // Gradual adjustments when close to the target
    int distance_error = abs(dist_ema - _DIST_TARGET);

    // If the distance error is large, make a more aggressive adjustment
    if (distance_error > _DIST_TOLERANCE) {
      if (dist_ema > (_DIST_TARGET + _DIST_TOLERANCE)) {
        duty_target = _DUTY_NEU + _BANGBANG_RANGE;
      } else if (dist_ema < (_DIST_TARGET - _DIST_TOLERANCE)) {
        duty_target = _DUTY_NEU - _BANGBANG_RANGE;
      }
    } 
    // If within tolerance, start fine adjustment for smooth stop
    else {
      // Fine-tune the servo movement gradually as it approaches the target
      int fine_adjustment = map(distance_error, 0, _DIST_TOLERANCE, 0, 50);  // Fine adjustments based on error
      if (dist_ema > _DIST_TARGET) {
        duty_target = _DUTY_NEU + fine_adjustment;
      } else if (dist_ema < _DIST_TARGET) {
        duty_target = _DUTY_NEU - fine_adjustment;
      } else {
        duty_target = _DUTY_NEU;  // Stop moving once it's within tolerance
      }
    }

    digitalWrite(PIN_LED, dist_ema < _DIST_TARGET ? HIGH : LOW);  // LED control based on distance
  }

  // Servo control event
  if (event_servo) {
    event_servo = false;

    // Gradually adjust the current duty towards the target duty for smooth servo motion
    if (duty_target > duty_current) {
      duty_current += duty_change_per_interval;
      if (duty_current > duty_target) duty_current = duty_target;
    } else if (duty_target < duty_current) {
      duty_current -= duty_change_per_interval;
      if (duty_current < duty_target) duty_current = duty_target;
    }

    // Ensure the duty stays within the limits
    if (duty_current < _DUTY_MIN) duty_current = _DUTY_MIN;
    else if (duty_current > _DUTY_MAX) duty_current = _DUTY_MAX;

    myservo.writeMicroseconds(duty_current);
  }

  // Serial output event
  if (event_serial) {
    event_serial = false;

    Serial.print("DIST: "); Serial.print(dist_ema);
    Serial.print(", duty_target: "); Serial.print(duty_target);
    Serial.print(", duty_current: "); Serial.println(duty_current);
  }
}

float volt_to_distance(int a_value) {
  return (6762.0 / (a_value - 9) - 4.0) * 10.0;  // Example conversion formula
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose) {
  unsigned int *ir_val, ret_val;
  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0) position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL) return 0;

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);

  return ret_val;
}
