// Volatile variables.
volatile unsigned long Last_Time_Interval = 0; // Previous recorded time interval between teeth detected, in microseconds, for missing tooth detection.
volatile unsigned long Last_Tooth_Time = 0; // Most recent recorded time when tooth was detected, in microseconds.
volatile byte Tooth_Number = 0; // Most recent detected tooth number, with 1 indicating TDC. A value of 0 indicates engine stop or sync loss.

// Nonvolatile variables.
unsigned long Injection_Start_Time; // Time when crankshaft reached injection angle and injector signal went high, in microseconds.
unsigned long Injection_Time = 5000; // Time calculated to hold injector signal high to inject correct fuel load, in microseconds.
float Crankshaft_Position; // Crankshaft position, based on last detected tooth number and speed-based interpolation, in degrees.
float Crankshaft_Speed; // Crankshaft angular speed in degrees per microsecond.
float Old_Crankshaft_Position = 0; // Crankshaft position last seen by loop function.
byte Old_Tooth_Number = 0; // Tooth number last seen by loop function. If != Tooth_Number, loop function knows new tooth was detected.

// Constants.
const byte Number_Of_Teeth = 24; // Number of teeth on trigger wheel. INCLUDE MISSING TEETH (e.g. enter 18 for 18-1 trigger wheel, not 17).
const byte Number_Of_Missing_Teeth = 2; // Number of missing teeth on trigger wheel.
const byte Number_Of_Actual_Teeth = Number_Of_Teeth - Number_Of_Missing_Teeth;
const float Degrees_Per_Tooth = 360/Number_Of_Teeth;
const float Gap_Detection_Threshold = 1.75; // If new interval is this many times longer than last interval, detect gap.
const float Injection_Angle = 90; // Crankshaft angle at which injection should occur, in degrees.

// Pin name constants.
#define Hall_Switch_Pin 3
#define Injector_Signal_Pin 14

void Trigger_Wheel_Tooth_ISR() {
  // Interrupt triggered when crank position signal falls from high to low.
  unsigned long Interrupt_Time = micros(); // Time tooth was just detected.
  unsigned long Current_Time_Interval = Interrupt_Time - Last_Tooth_Time; // Just recorded time interval between teeth detected, in microseconds.
  if (Current_Time_Interval > Gap_Detection_Threshold*Last_Time_Interval) {
    if ((Tooth_Number == Number_Of_Actual_Teeth) || (Tooth_Number == 0)) {
      Tooth_Number = 1;
    }
    else if ((Tooth_Number != Number_Of_Actual_Teeth) && (Tooth_Number != 0)) {
      Tooth_Number = 0;
    }
  }
  else if (Tooth_Number != 0) {
    if (Tooth_Number < Number_Of_Actual_Teeth) {
      Tooth_Number++;
    }
    else {
      Tooth_Number = 0;
    }
  }
  Last_Tooth_Time = Interrupt_Time;
  Last_Time_Interval = Current_Time_Interval;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(Hall_Switch_Pin, INPUT);
  pinMode(Injector_Signal_Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Hall_Switch_Pin), Trigger_Wheel_Tooth_ISR, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((Tooth_Number != Old_Tooth_Number) && (Tooth_Number != 0)) {
    noInterrupts();
    if (Tooth_Number != 1) {
      Crankshaft_Speed = Degrees_Per_Tooth/Last_Time_Interval;
    }
    else if (Tooth_Number == 1) {
      Crankshaft_Speed = (Number_Of_Missing_Teeth + 1)*(Degrees_Per_Tooth/Last_Time_Interval);
    }
    Crankshaft_Position = (Tooth_Number - 1)*Degrees_Per_Tooth;
    Old_Tooth_Number = Tooth_Number;
    interrupts();
  }
  if (((Tooth_Number != Number_Of_Actual_Teeth) && (Tooth_Number != 0) && (Crankshaft_Position < Tooth_Number*Degrees_Per_Tooth)) || 
  ((Tooth_Number == Number_Of_Actual_Teeth) && (Crankshaft_Position < 360))) {
    Crankshaft_Position = (Tooth_Number - 1)*Degrees_Per_Tooth + (micros() - Last_Tooth_Time)*Crankshaft_Speed;
  }
  if ((Old_Crankshaft_Position < Injection_Angle) && (Crankshaft_Position >= Injection_Angle)) {
    // Insert code to calculate fuel load here.
    digitalWrite(Injector_Signal_Pin, HIGH);
    Injection_Start_Time = micros();
  }
  if (micros() - Injection_Start_Time >= Injection_Time) {
    digitalWrite(Injector_Signal_Pin, LOW);
  }
  // Diagnostic crankshaft position signal.
  if (micros() % 10000 == 0) {
    Serial.println(Crankshaft_Position);
  }
  Old_Crankshaft_Position = Crankshaft_Position;
}
