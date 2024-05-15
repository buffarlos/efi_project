// Volatile variable.
volatile bool Tooth_Detected = false; // Trigger_Wheel_Tooth_ISR will set this value to true when tripped.

// Nonvolatile variables.
unsigned long Last_Time_Interval = 0; // Previous recorded time interval between teeth detected, in microseconds, for missing tooth detection.
unsigned long Last_Tooth_Time = 0; // Most recent recorded absolute time when tooth was detected, in microseconds.
unsigned long Injection_Start_Time; // Absolute time when crankshaft reached injection angle and injector signal went high, in microseconds.
unsigned long Injection_Time; // Time interval calculated to hold injector signal high to inject correct fuel load, in microseconds.
float Crankshaft_Position; // Crankshaft position, based on last detected tooth number and speed-based interpolation, in degrees.
float Crankshaft_Speed; // Crankshaft speed, in degrees per microsecond.
float Old_Crankshaft_Position = 0; // Crankshaft position last seen by loop function, in degrees.
float MAP; // Manifold absolute pressure, in kPa.
byte Tooth_Number = 0; // Most recent detected tooth number, with 1 corresponding to TDC. A value of 0 indicates engine stop or sync loss.
byte Old_Tooth_Number = 0; // Tooth number last seen by loop function. If != Tooth_Number, loop function knows new tooth was detected.

// Constants.
const byte Number_Of_Teeth = 24; // Number of teeth on trigger wheel. INCLUDE MISSING TEETH (e.g. enter 18 for 18-1 trigger wheel, not 17).
const byte Number_Of_Missing_Teeth = 2; // Number of missing teeth on trigger wheel.
const byte Number_Of_Actual_Teeth = Number_Of_Teeth - Number_Of_Missing_Teeth; // Number of physical teeth on trigger wheel.
const float Degrees_Per_Tooth = 360/Number_Of_Teeth; // Angle swept out by each trigger wheel tooth, in degrees.
const float Gap_Detection_Threshold = 1.75; // If new time interval between teeth is this many times longer than last interval, detect gap.
const float Injection_Angle = 90; // Crankshaft angle at which injection should occur, in degrees.
const float Redline = 0.0216; // Crankshaft speed beyond which fuel will be cut, in degrees per microsecond.

// Pin name constants.
#define Hall_Switch_Pin 9
#define Injector_Signal_Pin 6

void Trigger_Wheel_Tooth_ISR() {
  // Interrupt triggered when Hall effect switch signal falls from high to low.
  Tooth_Detected = true;
}

float Injection_Time_Calculation(float Crankshaft_Speed, float MAP) {
  // Takes current crankshaft speed and MAP, and calculates interpolated injection time based on VE table.
  // VE Table Constants.
  const int VE_Table_Speed_Points = 8; // Number of linearly spaced tabulated crankshaft speed points in VE table.
  const int VE_Table_MAP_Points = 8; // Number of linearly spaced tabulated MAP points in VE table.
  const float VE_Table_Minimum_Speed = 0.003; // Minimum tabulated crankshaft speed in VE table, in degrees per microsecond.
  const float VE_Table_Maximum_Speed = 0.024; // Maximum tabulated crankshaft speed in VE table, in degrees per microsecond.
  const float VE_Table_Speed_Step = (VE_Table_Maximum_Speed - VE_Table_Minimum_Speed)/(VE_Table_Speed_Points - 1); // Interval between tabulated crankshaft speed values.
  const float VE_Table_Minimum_MAP = 20.0; // Minimum tabulated MAP in VE table, in kPa.
  const float VE_Table_Maximum_MAP = 120.0; // Minimum tabulated MAP in VE table, in kPa.
  const float VE_Table_MAP_Step = (VE_Table_Maximum_MAP - VE_Table_Minimum_MAP)/(VE_Table_MAP_Points - 1); // Interval between tabulated MAP values.
  float VE_Table[VE_Table_MAP_Points][VE_Table_Speed_Points] = {
    {0.3, 0.3, 0.3, 0.3, 0.4, 0.4, 0.3, 0.3}, // 20.0 kPa
    {0.4, 0.4, 0.4, 0.5, 0.5, 0.5, 0.5, 0.4}, // 34.3 kPa
    {0.5, 0.6, 0.6, 0.7, 0.7, 0.7, 0.7, 0.7}, // 48.6 kPa
    {0.6, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.7}, // 62.9 kPa
    {0.6, 0.7, 0.7, 0.7, 0.8, 0.8, 0.8, 0.8}, // 77.1 kPa
    {0.7, 0.7, 0.7, 0.8, 0.8, 0.8, 0.8, 0.8}, // 91.4 kPa
    {0.7, 0.8, 0.8, 0.8, 0.8, 0.9, 0.9, 0.8}, // 105.7 kPa
    {0.7, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9, 0.8}  // 120.0 kPa
  // 500  1000 1500 2000 2500 3000 3500 4000
  }; // Main VE table.
  float Start_VE_Table[1][VE_Table_MAP_Points] = {
    {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}
  }; // Startup regime VE table.
  int Lower_MAP_Index = max(0, min(VE_Table_MAP_Points - 1, (int)((MAP - VE_Table_Minimum_MAP)/VE_Table_MAP_Step))); // Index of tabulated MAP below measured MAP.
  int Upper_MAP_Index = min(VE_Table_MAP_Points - 1, Lower_MAP_Index + 1); // Index of tabulated MAP above measured MAP.
  float Interpolated_VE; // Interpolated VE based on VE table.
  if (Crankshaft_Speed < VE_Table_Minimum_Speed) {
    Interpolated_VE = Interpolation(MAP, (Lower_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP), (Upper_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP),
      Start_VE_Table[0][Lower_MAP_Index], Start_VE_Table[0][Upper_MAP_Index]);
  }
  else {
    int Lower_Speed_Index = max(0, min(VE_Table_Speed_Points - 1, (int)((Crankshaft_Speed - VE_Table_Minimum_Speed)/VE_Table_Speed_Step))); // Index of tabulated crankshaft speed below measured crankshaft speed.
    int Upper_Speed_Index = min(VE_Table_Speed_Points - 1, Lower_Speed_Index + 1); // Index of tabulated crankshaft speed above measured crankshaft speed.
    Interpolated_VE = Interpolation(Crankshaft_Speed, (Lower_Speed_Index*VE_Table_Speed_Step + VE_Table_Minimum_Speed), (Upper_Speed_Index*VE_Table_Speed_Step + VE_Table_Minimum_Speed),
      Interpolation(MAP, (Lower_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP), (Upper_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP),
        VE_Table[Lower_MAP_Index][Lower_Speed_Index], VE_Table[Lower_MAP_Index][Upper_Speed_Index]),
      Interpolation(MAP, (Lower_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP), (Upper_MAP_Index*VE_Table_MAP_Step + VE_Table_Minimum_MAP),
        VE_Table[Upper_MAP_Index][Lower_Speed_Index], VE_Table[Upper_MAP_Index][Upper_Speed_Index]));
  }
  // Insert code to translate VE to an injector pulse length.
  return Interpolated_VE;
}

float Interpolation(float x, float xl, float xu, float yl, float yu) {
  // Interpolation function for VE table.
  return yl + (yu - yl)*(x - xl)/(xu - xl);
}

void setup() {
  pinMode(Hall_Switch_Pin, INPUT);
  pinMode(Injector_Signal_Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Hall_Switch_Pin), Trigger_Wheel_Tooth_ISR, FALLING);
}

void loop() {
  if (Tooth_Detected == true) {
    unsigned long Interrupt_Time = micros(); // Absolute time at which tooth was just detected, in microseconds.
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
    Tooth_Detected = false;
  }
  if ((Tooth_Number != Old_Tooth_Number) && (Tooth_Number != 0)) {
    if (Tooth_Number != 1) {
      Crankshaft_Speed = Degrees_Per_Tooth/Last_Time_Interval;
    }
    else if (Tooth_Number == 1) {
      Crankshaft_Speed = (Number_Of_Missing_Teeth + 1)*(Degrees_Per_Tooth/Last_Time_Interval);
    }
    Crankshaft_Position = (Tooth_Number - 1)*Degrees_Per_Tooth;
    Old_Tooth_Number = Tooth_Number;
  }
  if (((Tooth_Number != Number_Of_Actual_Teeth) && (Tooth_Number != 0) && (Crankshaft_Position < Tooth_Number*Degrees_Per_Tooth)) || 
  ((Tooth_Number == Number_Of_Actual_Teeth) && (Crankshaft_Position < 360))) {
    Crankshaft_Position = (Tooth_Number - 1)*Degrees_Per_Tooth + (micros() - Last_Tooth_Time)*Crankshaft_Speed;
  }
  if ((Old_Crankshaft_Position < Injection_Angle) && (Crankshaft_Position >= Injection_Angle) && (Crankshaft_Speed < Redline) && (Tooth_Number != 0)) {
    // Insert code to translate MAP input value to MAP.
    MAP = 95;
    Injection_Time = 10000*Injection_Time_Calculation(Crankshaft_Speed, MAP);
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
