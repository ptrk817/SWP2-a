#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  Serial.println("Hello World");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // Turn off LED
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); // Update the toggle state
  digitalWrite(PIN_LED, toggle); // Update LED status
  delay(1000); // Wait for 1,000 milliseconds
}

int toggle_state(int currentState) {
  return !currentState; // Toggle state (0 -> 1, 1 -> 0)
}
