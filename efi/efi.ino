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
float MAP; // Manifold absolute pressure, in kilopascals.
float Old_MAP = 101.0; // Last measured manifold absolute pressure, in kilopascals.
float Tooth_Start_Position; // Angular position of last detected tooth, in degrees.
float Angle_Interpolation_Limit; // Angle to which crankshaft position should be interpolated.
byte Tooth_Number = 0; // Most recent detected tooth number, with 1 corresponding to TDC. A value of 0 indicates engine stop or sync loss.

// Constants.
const byte NUMBER_OF_TEETH = 22; // Number of teeth on trigger wheel. DON'T INCLUDE MISSING TEETH (e.g. enter 22 for 24-2 trigger wheel, not 24).
const byte NUMBER_OF_MISSING_TEETH = 2; // Number of missing teeth on trigger wheel.
const float DEGREES_PER_TOOTH = 360/(NUMBER_OF_TEETH + NUMBER_OF_MISSING_TEETH); // Angle swept out by each trigger wheel tooth, in degrees.
const float GAP_DETECTION_THRESHOLD = 1.75; // If new time interval between teeth is this many times longer than last interval, detect gap.
const float INJECTION_ANGLE = 90.0; // Crankshaft angle at which injection should occur, in degrees.
const float REDLINE = 0.0216; // Crankshaft speed beyond which fuel will be cut, in degrees per microsecond.
const float ENRICHMENT_FACTOR = 1.5; // Multiply injector pulse length by this amount so long as choke switch is on.

// MAP sensor calibration constants.
const float MAP_SLOPE = 0.11943; // Slope of MAP sensor output, in kilopascals per count.
const float MAP_CAL_PRESSURE = 101.0; // MAP sensor calibration pressure point, in kilopascals.
const int MAP_CAL_COUNT = 817; // MAP sensor output at calibration pressure point, in counts.

// Injector constants.
const float DEAD_TIME = 1000.0; // Minimum injector pulse length to open injector, in microseconds.
const unsigned long ONE_HUNDRED_PCT_VE_PULSE = 8694; // Stoichiometric fuel load pulse at 100% VE minus dead time, in microseconds.

// Pin name constants.
#define Hall_Switch_Pin 9
#define Injector_Signal_Pin 6
#define Choke_Pin 21

void Trigger_Wheel_Tooth_ISR() {
  // Interrupt triggered when Hall effect switch signal falls from high to low.
  Tooth_Detected = true;
}

unsigned long Injection_Time_Calculation(float Crankshaft_Speed, float MAP) {
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
  // Translate VE to an injector pulse length.
  float Injection_Time = DEAD_TIME + (ONE_HUNDRED_PCT_VE_PULSE*Interpolated_VE);
  if (digitalRead(Choke_Pin) == LOW) {
    Injection_Time = Injection_Time*ENRICHMENT_FACTOR;
  }
  int Int_Injection_Time = static_cast<int>(Injection_Time);
  return Int_Injection_Time;
}

float Interpolation(float x, float xl, float xu, float yl, float yu) {
  // Interpolation function for VE table.
  return yl + (yu - yl)*(x - xl)/(xu - xl);
}

void setup() {
  pinMode(Hall_Switch_Pin, INPUT);
  pinMode(Injector_Signal_Pin, OUTPUT);
  pinMode(Choke_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Hall_Switch_Pin), Trigger_Wheel_Tooth_ISR, FALLING);
}

void loop() {
  // Pass volatile tooth detection flag to nonvolatile tooth detection flag in interrupt-safe environment.
  noInterrupts();
  bool Need_Update = Tooth_Detected;
  Tooth_Detected = false;
  interrupts();
  // Update tooth counter if tooth detection flag tripped.
  if (Need_Update) {
    unsigned long Interrupt_Time = micros(); // Absolute time at which tooth was just detected, in microseconds.
    unsigned long Current_Time_Interval = Interrupt_Time - Last_Tooth_Time; // Just recorded time interval between teeth detected, in microseconds.
    if (Current_Time_Interval > GAP_DETECTION_THRESHOLD*Last_Time_Interval) {
      if ((Tooth_Number == NUMBER_OF_TEETH) || (Tooth_Number == 0)) {
        Tooth_Number = 1;
      }
      else if ((Tooth_Number != NUMBER_OF_TEETH) && (Tooth_Number != 0)) {
        Tooth_Number = 0;
      }
    }
    else if (Tooth_Number != 0) {
      if (Tooth_Number < NUMBER_OF_TEETH) {
        Tooth_Number++;
      }
      else {
        Tooth_Number = 0;
      }
    }
    Last_Tooth_Time = Interrupt_Time;
    Last_Time_Interval = Current_Time_Interval;
    if (Tooth_Number != 0) {
      Tooth_Start_Position = (Tooth_Number - 1)*DEGREES_PER_TOOTH;
      if (Tooth_Number != NUMBER_OF_TEETH) {
        Angle_Interpolation_Limit = Tooth_Number*DEGREES_PER_TOOTH;
      }
      else if (Tooth_Number == NUMBER_OF_TEETH) {
        Angle_Interpolation_Limit = 360.0;
      }
      if (Tooth_Number != 1) {
        Crankshaft_Speed = DEGREES_PER_TOOTH/Last_Time_Interval;
      }
      else if (Tooth_Number == 1) {
        Crankshaft_Speed = (NUMBER_OF_MISSING_TEETH + 1)*(DEGREES_PER_TOOTH/Last_Time_Interval);
      }
      Crankshaft_Position = Tooth_Start_Position;
    }
    Need_Update = false;
  }
  // Interpolate crankshaft position based on last calculated crankshaft speed and time elapsed since last tooth detection.
  if (Crankshaft_Position < Angle_Interpolation_Limit) {
    Crankshaft_Position = Tooth_Start_Position + (micros() - Last_Tooth_Time)*Crankshaft_Speed;
    if (Crankshaft_Position > Angle_Interpolation_Limit) {
      Crankshaft_Position = Angle_Interpolation_Limit;
    }
  }
  // Fuel injection.
  if ((Old_Crankshaft_Position < INJECTION_ANGLE) && (Crankshaft_Position >= INJECTION_ANGLE) && (Crankshaft_Speed < REDLINE) && (Tooth_Number != 0)) {
    // Insert code to translate MAP input value to MAP.
    int MAP_Raw = analogRead(A4);
    MAP = MAP_CAL_PRESSURE + ((MAP_Raw - MAP_CAL_COUNT)*MAP_SLOPE);
    Injection_Time = Injection_Time_Calculation(Crankshaft_Speed, min(MAP, Old_MAP));
    digitalWrite(Injector_Signal_Pin, HIGH);
    Injection_Start_Time = micros();
    Old_MAP = MAP;
    // Diagnostic injection pulse length.
    /*
    Serial.println(Injection_Time);
    */
  }
  if (micros() - Injection_Start_Time >= Injection_Time) {
    digitalWrite(Injector_Signal_Pin, LOW);
  }
  // Diagnostic crankshaft position signal.
  /*
  if (micros() % 10000 == 0) {
    Serial.println(Crankshaft_Position);
  }
  */
  // Update old crankshaft position.
  Old_Crankshaft_Position = Crankshaft_Position;
}
