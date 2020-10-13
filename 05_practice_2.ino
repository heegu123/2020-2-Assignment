#define PIN_LED 7

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  digitalWrite(PIN_LED, 0); // active low공부.
  delay(1000);
}

void loop() {
  for(int i=1; i<=5; i++){
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }
  while(1){
    digitalWrite(PIN_LED,1);
    }
}
