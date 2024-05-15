unsigned int number_of_pulses = 100; // How many injector pulses to test.
unsigned int pulse_length = 10000; // Injector pulse length, in microseconds.
unsigned int rpm = 3600; // Engine speed to simulate, in revolutions per minute.
unsigned int cycle_length; // Time to execute one engine cycle, in microseconds.
unsigned int off_time; // Time between injection close and open, in microseconds.

unsigned int counter = 0; // Number of injector cycles completed.
bool started = false; // Indicates whether test parameters have been received and test has started.

#define Injector_Signal_Pin 6

void setup() {
  pinMode(Injector_Signal_Pin, OUTPUT);
}

void loop() {
  if (Serial.peek() >= 0) {
    number_of_pulses = Serial.parseInt();
    pulse_length = Serial.parseInt();
    rpm = Serial.parseInt();
    cycle_length = 1000000/(rpm/60);
    off_time = cycle_length - pulse_length;
    Serial.println("######### Current test parameters #########");
    Serial.println("Number of injection events: ");
    Serial.println(number_of_pulses);
    Serial.println("Pulse length (microseconds): ");
    Serial.println(pulse_length);
    Serial.println("Engine RPM: ");
    Serial.println(rpm);
    Serial.println("Calculated engine cycle length (microseconds): ");
    Serial.println(cycle_length);
    Serial.println("Calculated off time (microseconds): ");
    Serial.println(off_time);
    Serial.read();
    started = true;
  }
  if ((counter <= number_of_pulses) && (started == true)) {
    digitalWrite(Injector_Signal_Pin, HIGH);
    delayMicroseconds(pulse_length);
    digitalWrite(Injector_Signal_Pin, LOW);
    delayMicroseconds(off_time);
    counter += 1;
    if (counter >= number_of_pulses) {
      started = false;
      counter = 0;
    }
  }
}
