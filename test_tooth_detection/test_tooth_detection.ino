#define Hall_Switch_Pin 3
#define Output_LED_Pin 14

void Trigger_Wheel_Tooth_Interrupt() {
  if (digitalRead(Hall_Switch_Pin) == HIGH) {
    digitalWrite(Output_LED_Pin, LOW);
  }
  else {
    digitalWrite(Output_LED_Pin, HIGH);
  }
}

void setup() {
  pinMode(Output_LED_Pin, OUTPUT);
  pinMode(Hall_Switch_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Hall_Switch_Pin), Trigger_Wheel_Tooth_Interrupt, CHANGE);
}

void loop() {
}
