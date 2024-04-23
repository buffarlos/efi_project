// Volatile variables, passed between interrupt function and loop function.
volatile unsigned long Current_Time_Gap = 0; // Most recent recorded time gap between teeth detected, in microseconds.
volatile unsigned long Last_Time_Gap = 0; // Previous recorded time gap between teeth detected, in microseconds, for missing tooth detection.
volatile unsigned long Last_Tooth_Time = 0; // Most recent recorded time when tooth was detected, in microseconds.
volatile byte Tooth_Number = 0; // Most recent detected tooth number, with 1 indicating TDC. A value of 0 indicates engine stop or sync loss.

// Nonvolatile variables, calculated and used only by loop function.
unsigned long Injection_Start_Time = 0; // Time when crankshaft reached injection angle and injector signal went high, in microseconds.
unsigned long Injection_Time = 20000; // Time calculated to hold injector signal high to inject correct fuel load, in microseconds.
float Crankshaft_Position; // Crankshaft position, based on last detected tooth number and speed-based interpolation, in degrees.
byte Old_Tooth_Number = 0; // Tooth number last seen by loop function. If modified by interrupt, loop function knows a new tooth was detected.
bool Injecting = false; // Whether injector signal is high or not.

// Constants.
const unsigned long Engine_Stop_Threshold = 44444; // If time between teeth is longer than this threshold in microseconds, detect engine stop.
const byte Number_Of_Teeth = 18; // Number of teeth on trigger wheel. INCLUDE MISSING TEETH (e.g. enter 18 for 18-1 trigger wheel, not 17).
const byte Number_Of_Missing_Teeth = 1; // Number of missing teeth on trigger wheel.
const byte Number_Of_Actual_Teeth = Number_Of_Teeth - Number_Of_Missing_Teeth;
const float Degrees_Per_Tooth = 360/Number_Of_Teeth;

// Pin name constants.
#define Hall_Switch_Pin 3
#define Injector_Signal_Pin 14

void Trigger_Wheel_Tooth_ISR() {
  // Interrupt triggered when crank position signal falls from high to low.
  unsigned long Interrupt_Time = micros();
  Current_Time_Gap = Interrupt_Time - Last_Tooth_Time;
  if (Current_Time_Gap > (Number_Of_Missing_Teeth + 0.5)*Last_Time_Gap) {
    if ((Tooth_Number == Number_Of_Actual_Teeth) || (Tooth_Number == 0)) {
      Tooth_Number = 1;
    }
    else if ((Tooth_Number != Number_Of_Actual_Teeth) && (Tooth_Number != 0)) {
      Tooth_Number = 0;
      Serial.println("False gap!");
    }
  }
  else if (Tooth_Number != 0) {
    if (Tooth_Number < Number_Of_Actual_Teeth) {
      Tooth_Number++;
    }
    else {
      Tooth_Number = 0;
      Serial.println("Gap not detected where expected!");
    }
  }
  if ((Tooth_Number != Old_Tooth_Number) and (Tooth_Number != 0)) {
    Serial.println(Tooth_Number);
  }
  Last_Tooth_Time = Interrupt_Time;
  Last_Time_Gap = Current_Time_Gap;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(Hall_Switch_Pin, INPUT);
  pinMode(Injector_Signal_Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Hall_Switch_Pin), Trigger_Wheel_Tooth_ISR, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Old_Tooth_Number = Tooth_Number;
}
