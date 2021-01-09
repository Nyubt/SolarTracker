#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS3231.h>
#include <sunMoon.h>
#include <Stepper.h>

//Pins
#define STEPPER_UPPER_PIN_1 5
#define STEPPER_UPPER_PIN_2 6
#define STEPPER_UPPER_PIN_3 7
#define STEPPER_UPPER_PIN_4 8
#define STEPPER_LOWER_PIN_1 9
#define STEPPER_LOWER_PIN_2 10
#define STEPPER_LOWER_PIN_3 11
#define STEPPER_LOWER_PIN_4 12
int voltageInputPin = A3;

// number of steps on the lower motor
#define STEPS 200

Stepper motor(STEPS, STEPPER_LOWER_PIN_4, STEPPER_LOWER_PIN_3, STEPPER_LOWER_PIN_2, STEPPER_LOWER_PIN_1);

// I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69. Else it is 0x68
const int MPU_ADDR = 0x69;

//Input variables
//Gratiesti coordinates
float Latitude = 47.100790; 
float Longitude = 28.826520;
int Timezone = 120; // in minutes
float UpperStep = 0.7; //Was determined experimentally
float LowerStep = 1.8; //+-5% error
float CurentAzimuth = 90;
float CurentElevation = 45;
int StepNumber = 0;
bool rest = true;
time_t CurrentTime = 0;
time_t OldTime = 0;
time_t sRise;
time_t sSet;

//Program Variables
float ZenithAngle;
float Azimuth = 90;
float ElevationAngle;
int Charge = 0;

//Library defined variables
DS3231  rtc(SDA, SCL);
Time  t;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  rtc.begin();  
  //Loop until curent time found
  while(!getDate()){
    Serial.println("Error: Time not found");
  }
  OldTime = CurrentTime - 15 * 60;
  pinMode(STEPPER_UPPER_PIN_1, OUTPUT);  
  pinMode(STEPPER_UPPER_PIN_2, OUTPUT);  
  pinMode(STEPPER_UPPER_PIN_3, OUTPUT);  
  pinMode(STEPPER_UPPER_PIN_4, OUTPUT);
  pinMode(STEPPER_LOWER_PIN_1, OUTPUT);  
  pinMode(STEPPER_LOWER_PIN_2, OUTPUT);  
  pinMode(STEPPER_LOWER_PIN_3, OUTPUT);  
  pinMode(STEPPER_LOWER_PIN_4, OUTPUT);
  getCurrentElevationAngle();
  motor.setSpeed(1);
  delay(30000);
}

void loop() {  
  //Loop until curent time found
  while(!getDate()){
    Serial.println("Error: Time not found");
  }
  readVoltmeterValue();
  readLatitudeLongitude();  
  sunTimeCalc(); //Run sunrise and sunset calculations  
  sunPosition(); //Run sun position calculations
  
  //Check if the sun is up
  if(CurrentTime >= sRise && CurrentTime <= sSet) {    
    rest = false; //Set rest period as false
    
    //Check that 15 minutes have passed since last positioning
    if(CurrentTime - OldTime >= 15 * 60) {
      
      //Check if elevation angle has changed, 
      if (CurentElevation != ElevationAngle) {       
        changeElevationAngle(); //Change elevation angle
      }  
      
      //Check if azimuth angle has changed
      if (CurentAzimuth != Azimuth) {       
        changeAzimuthAngle(); //Change azimuth angle
      }      
      OldTime = CurrentTime; //Update time of last positioning
    }
  } 
  //Check if sun is down and PV panel is still active
  else if (CurrentTime > sSet && rest == false) {    
    Azimuth = 90; //Change azimuth angle due east
    changeAzimuthAngle();
    ElevationAngle = 75; //Change elevation to 15 degrees
    changeElevationAngle();    
    rest = true; //Set rest period as true
  }
  Serial.println(generateNodeRedString());
  delay(5000);
}

//Read battery charge
void readVoltmeterValue(){
  float voltageValue = analogRead(voltageInputPin);
  float vout = (voltageValue * 5.0) / 1024.0;
  Charge = mapFloatValues(vout, 0, 3.6, 0, 100);
}

//Map function for float values
int mapFloatValues(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Read current elevation value from MPU-6050 
void getCurrentElevationAngle() {
  int count = 0;
  float elevAngle = 0;
  float AccX, AccY, AccZ;
  float accAngleX, accAngleY;
  while(count < 5){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 7*2, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value    
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error() custom function for more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    elevAngle += accAngleY;
    count++;
  }
  CurentElevation = elevAngle/5; //Get average elevation angle
}

//Calculate sunrise and sunset
void sunTimeCalc() {
  sunMoon  sm;
  time_t s_date = rtc.getUnixTime(rtc.getTime());
  sm.init(Timezone, Latitude, Longitude);  
  sRise = sm.sunRise(s_date);
  sSet  = sm.sunSet(s_date);  
}

//Convert degrees to radians
double deg2rad(double deg) {
    return (2 * PI * deg) / 360;
}

//Calculate sun position
void sunPosition(){  
  double lat_deg = Latitude;
  double long_deg = Longitude;
  double timezone = Timezone / 60;  
  double day_of_year = calculateDayOfYear(t.date, t.mon, t.year);
  double hours = t.hour;
  double min = t.min;
  double sec = t.sec;
  int num_of_days = IsLeapYear(t.year) ? 366 : 365;
  
  double gamma = (2 * M_PI / num_of_days) * (day_of_year + hours / 24 + min / 24 / 60);
  double eqtime_deg = 229.18 * (0.000075 + 0.001868 * cos(gamma) - 0.032077 * sin(gamma) - 0.014615 * cos(2 * gamma) - 0.040849 * sin(2 * gamma));
  double decl = 0.006918 - 0.399912 * cos(gamma) + 0.070257 * sin(gamma) - 0.006758 * cos(2 * gamma) + 0.000907 * sin(2 * gamma) - 0.002697 * cos(3 * gamma) + 0.00148 * sin(3 * gamma);
  double time_offset = eqtime_deg + 4 * long_deg - 60 * timezone;
  double tst_deg = hours * 60 + min + sec / 60 + time_offset;
  double ha_deg = (tst_deg / 4) - 180;
  
  double ha = deg2rad(ha_deg);
  double lat = deg2rad(lat_deg);
  double lng = deg2rad(long_deg);
  
  double phi = acos(sin(lat) * sin(decl) + cos(lat) * cos(decl) * cos(ha));
  double ZenithAngle = 360 * phi / (2 * PI);
  
  double compl_theta = acos(-(sin(lat) * cos(phi) - sin(decl)) / (cos(lat) * sin(phi)));
  double theta;
  if (hours < 12){
    theta = compl_theta;
  } else {
    theta = 2 * M_PI - compl_theta;
  }
  ElevationAngle = ZenithAngle; //We chose the Y angle of out MPU6050 to show elevation, so we change equation accordingly. Otherwise: ElevationAngle = theta; 
  Azimuth = 360 * theta / (2 * PI);
}

//Check for leap year
bool IsLeapYear(int year)
{
  if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0) {
    return 1;  // Is a leap year
  } else {
    return 0; // not a leap year
  }
}

int calculateDayOfYear(int day, int month, int year) {
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};  
  if (IsLeapYear(year)) {
    daysInMonth[1] = 29;
  }
  int days = 0;
  for (int i = 0; i < month - 1; i++) {
    days += daysInMonth[i];
  }  
  days += day;
  return days;
}

//Get current time from RTC
bool getDate() {
  int m, d, y;
  t = rtc.getTime();
  CurrentTime = rtc.getUnixTime(t);
  const char* str = rtc.getDateStr();
  
  //Function used only to signal error with RTC
  if (sscanf(str, "%d.%d.%d", &m, &d, &y) != 3) {
    return false;
  }
  
  return true;
}

//Set tracker's elevation angle to real elevation angle
void changeElevationAngle(){  
  if(CurentElevation > ElevationAngle){
    while(CurentElevation - UpperStep >= ElevationAngle && CurentElevation - UpperStep > 2){
      stepUpperMotor(true);
      delay(20);
      getCurrentElevationAngle();
    }
  } else if(CurentElevation < ElevationAngle) {
      while(CurentElevation + UpperStep <= ElevationAngle && CurentElevation + UpperStep < 88){
        stepUpperMotor(false);
        delay(20);
        getCurrentElevationAngle();
      }
  }
  delay(1000);
}

//Moving motor using a half step 
void stepUpperMotor(bool dir){
  if(dir){
      switch(StepNumber){
        case 0:
          digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_2, LOW);
          digitalWrite(STEPPER_UPPER_PIN_3, LOW);
          digitalWrite(STEPPER_UPPER_PIN_4, LOW);
          break;
        case 1:
          digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_3, LOW);
          digitalWrite(STEPPER_UPPER_PIN_4, LOW);
          break;
        case 2:
          digitalWrite(STEPPER_UPPER_PIN_1, LOW);
          digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_3, LOW);
          digitalWrite(STEPPER_UPPER_PIN_4, LOW);
          break;
        case 3:
          digitalWrite(STEPPER_UPPER_PIN_1, LOW);
          digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_4, LOW);
          break;
        case 4:
          digitalWrite(STEPPER_UPPER_PIN_1, LOW);
          digitalWrite(STEPPER_UPPER_PIN_2, LOW);
          digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_4, LOW);
          break;
        case 5:
          digitalWrite(STEPPER_UPPER_PIN_1, LOW);
          digitalWrite(STEPPER_UPPER_PIN_2, LOW);
          digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
          break;
        case 6:
          digitalWrite(STEPPER_UPPER_PIN_1, LOW);
          digitalWrite(STEPPER_UPPER_PIN_2, LOW);
          digitalWrite(STEPPER_UPPER_PIN_3, LOW);
          digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
          break;
        case 7:
          digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
          digitalWrite(STEPPER_UPPER_PIN_2, LOW);
          digitalWrite(STEPPER_UPPER_PIN_3, LOW);
          digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
          break;
        } 
    } else {
      switch(StepNumber){
      case 0:
        digitalWrite(STEPPER_UPPER_PIN_1, LOW);
        digitalWrite(STEPPER_UPPER_PIN_2, LOW);
        digitalWrite(STEPPER_UPPER_PIN_3, LOW);
        digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_UPPER_PIN_1, LOW);
        digitalWrite(STEPPER_UPPER_PIN_2, LOW);
        digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
        break;
      case 2:
        digitalWrite(STEPPER_UPPER_PIN_1, LOW);
        digitalWrite(STEPPER_UPPER_PIN_2, LOW);
        digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_UPPER_PIN_1, LOW);
        digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_4, LOW);
        break;
      case 4:
        digitalWrite(STEPPER_UPPER_PIN_1, LOW);
        digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_3, LOW);
        digitalWrite(STEPPER_UPPER_PIN_4, LOW);
        break;
      case 5:
        digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_3, LOW);
        digitalWrite(STEPPER_UPPER_PIN_4, LOW);
        break;
      case 6:
        digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_2, LOW);
        digitalWrite(STEPPER_UPPER_PIN_3, LOW);
        digitalWrite(STEPPER_UPPER_PIN_4, LOW);
        break;
      case 7:
        digitalWrite(STEPPER_UPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_UPPER_PIN_2, LOW);
        digitalWrite(STEPPER_UPPER_PIN_3, LOW);
        digitalWrite(STEPPER_UPPER_PIN_4, HIGH);
        break;
      }
    }
    StepNumber++;
    
    if(StepNumber > 7){
      StepNumber = 0;
    }
}

//Set tracker Azimuth angle to real Azimuth angle
void changeAzimuthAngle(){
  if(CurentAzimuth > Azimuth){
    while(CurentAzimuth - LowerStep >= Azimuth){
      motor.step(-1);
      delay(20);
      CurentAzimuth = CurentAzimuth - LowerStep;
    }
  } else if(CurentAzimuth < Azimuth) {
    while(CurentAzimuth + LowerStep <= Azimuth){
      motor.step(1);
      delay(20);
      CurentAzimuth = CurentAzimuth + LowerStep;
    }
  }
  delay(1000);
}

//Concatenate string for serial transmission 
String generateNodeRedString()
{
  char sunrise[6], sunset[6];
  snprintf(sunrise, sizeof(sunrise), "%02d:%02d", hour(sRise), minute(sRise));
  snprintf(sunset, sizeof(sunset), "%02d:%02d", hour(sSet), minute(sSet));
  String s="lat/";  
  s+=String(Latitude);
  s+=";long/";
  s+=String(Longitude);
  s+=";rasarit/";
  s+=String(sunrise);
  s+=";apus/";
  s+=String(sunset);
  s+=";azimut/";
  s+=String(Azimuth);
  s+=";altitudinea/";
  s+=String(90 - ElevationAngle);
  s+=";azimutCurent/";
  s+=String(CurentAzimuth);
  s+=";altitudineCurenta/";
  s+=String(90 - CurentElevation);
  s+=";acumulator/";
  s+=String(Charge);
  return s;
}

//Read and overwrite Latitude and Longitude with incoming data
void readLatitudeLongitude(){
  String incomingString="";
  
  if(Serial.available()){
    char d = Serial.read();
    while (Serial.available()) {
      delay(3);
      if (Serial.available() > 0) {
        char c = Serial.read();
        incomingString += c;
      }
    }
    if(d == 'A'){
      Latitude = incomingString.toFloat();
    } else if(d == 'B'){
      Longitude = incomingString.toFloat();
    }    
  }

  incomingString = "";
}
