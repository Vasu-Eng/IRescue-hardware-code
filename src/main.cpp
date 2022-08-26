#include <SoftwareSerial.h>
#include <Wire.h>
#include "OTA.h"
#define LED 2
#define Ft 2.00
#define Fs 2.00

//******* registers *******

#define ADXL345_BW_RATE 0x2C
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FULL_RES 0x03

//******* variables ***********
uint32_t entry;//ota
int alert_status = 0, i = 0, flag = 0;
String ReadString, Device_id = "iRQ01";
float Latitude, Longtitude, LatInDeg, LongInDeg;
float F[50], T[50], f[50], t = 0;
const byte interruptPin = 0;
volatile bool gprs_data_status = 0;

SoftwareSerial gps_serial(14, 16);
SoftwareSerial gprsSerial(13, 12);
IRAM_ATTR void INT()
{ // for ESP interrupt
  gprs_data_status = !gprs_data_status;
}

//*******  custom functions initialization *******

bool writeRegister(byte addr, byte data);
byte readRegister(byte addr);
bool adxl_init();

void adxl345_sleep();
void adxl345_wakeup();

void alert_algo();
bool decision();
float gF(byte addr_0, byte addr_1);
float gF_net(float gx, float gy, float gz);
float bubbleSort(float arr[], int num);
void copy(float arr1[], float arr2[], int num);
float avg(float arr[], int index);
float sum(float arr[], int index);
float Area(float arr1[], float arr2[], int index);
int index_finder(float arr[], float val, int num);

void gprs_data_transfer(float Lat, float Long);
void gprs_handle(float Lat, float Long);

float rT(char x);
void ShowSerialData();

bool gps_data_handle();
float ConvertData(float RawDegrees);


void setup() {
  Serial.begin(38400);
  Serial.println("Booting");
  setupOTA("iRQ01","mrtechb0yWiFi","Sagar@2001");
//  setupOTA("iRQ01","Redmi","0987654321");
 // setupOTA("iRQ01","udit_home_network","home@tidu");
  // Your setup code
 gps_serial.begin(9600); // gps_serial port connected to GPS
  gprsSerial.begin(9600);
  Wire.begin();
  if (!adxl_init())
  {
    Serial.println("ADXL345 not connected");
  }
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), INT, FALLING);
}

void loop() {
   ArduinoOTA.handle();
  // Your code here
   alert_algo(); // if there is the possibillity of the alert it assign the flag =  1
  if (flag == 1)
  {
    flag = 0;
    Serial.println("<<<< Algo Ends ----");
    Serial.println(" ------- gForce  :  Time ---------------");
    for (int j = 0; j < i; j++)
    {
      Serial.print("----  ");
      Serial.print(F[j]);
      Serial.print("   :   ");
      Serial.print(T[j]);
      Serial.println("----  ");
    }
    if (decision())
    {
      alert_status = 1;
    }
  }
  if (alert_status == 1)
  {
    Serial.println("alert status : 1");
    rT('s');
    while (true)
    {
      digitalWrite(LED, HIGH);
      ArduinoOTA.handle();
      if (gps_data_handle())
      {
        digitalWrite(LED, HIGH);
        ArduinoOTA.handle();
        break;
      }
      Serial.print(" GPS runTime : ");
      Serial.println(rT('e'));
      digitalWrite(LED, LOW);
    }
    Serial.print("Latitude : ");
    Serial.println(Latitude);
    Serial.print("Longtitude : ");
    Serial.println(Longtitude);
    gprs_handle(Latitude, Longtitude); // it set the gprs_data_status = 1(using Ring pin as INT ) if the data has been sent to the server
  }
  if (gprs_data_status == 1)
  {
    digitalWrite(LED, LOW);
    Serial.print("gprs_data_status : ");
    gprs_data_status = 0;
    alert_status = 0;
  }

}




//----------------------------------------------------- Custom Functions definations ----------------------------------------------------------

bool writeRegister(byte addr, byte data)
{
  Wire.beginTransmission(0x53); // start the tramission __(slave_address)
  Wire.write(addr);             // BW_RATE registor - (data rate and power control)
  Wire.write(data);             // set the data rate to 25 hz   800 = 1110 , 400 = 1100 , 200 = 1011 , 100 = 1010
  return !Wire.endTransmission();
}

byte readRegister(byte addr)
{
  Wire.beginTransmission(0x53);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(0x53, 1);
  return Wire.read();
}

bool adxl_init()
{
  writeRegister(ADXL345_POWER_CTL, 0);
  writeRegister(ADXL345_POWER_CTL, 0b00001000); // prev = 48 ,  setMeasureMode(true);
  writeRegister(ADXL345_DATA_FORMAT, 0);
  writeRegister(ADXL345_BW_RATE, 0b00001111);     // data rate 3200 hz
  writeRegister(ADXL345_DATA_FORMAT, 0b00001011); // set the full resolution , +- 16g
  if (!((readRegister(ADXL345_DATA_FORMAT)) & (1 << ADXL345_FULL_RES)))
  {
    return false;
  }
  return true;
}

void adxl345_sleep()
{
  writeRegister(ADXL345_POWER_CTL, 0);          // standByMode
  writeRegister(ADXL345_POWER_CTL, 0b00000111); // sleep mode with minimum frequency of 1 Hz
}

void adxl345_wakeup()
{
  writeRegister(ADXL345_POWER_CTL, 0);          // standByMode
  writeRegister(ADXL345_POWER_CTL, 0b00001000); // Ready for measurements
}

void alert_algo()
{
  while (true)
  {
    rT('s');
    float gForce = gF_net(gF(ADXL345_DATAX0, ADXL345_DATAX1), gF(ADXL345_DATAY0, ADXL345_DATAY1), gF(ADXL345_DATAZ0, ADXL345_DATAZ1));
    if (gForce >= Ft)
    {
      Serial.print("  Algo running >>> ");
      F[i] = gForce;
      T[i] = rT('e');
      i++; // i is the ier variable
      Serial.println(i);
      flag = 1;
    }
    else
    {
      if (flag == 0)
      {
        Serial.print("-----------     Algo not trigged   !!  ");
        Serial.println(rT('e'));
      }
      break;
    }
  }
  ArduinoOTA.handle();
}

float gF(byte addr_0, byte addr_1)
{
  byte t0 = readRegister(addr_0);
  byte t1 = readRegister(addr_1);
  t1 = t1 & 0x03;
  uint16_t t = (t1 << 8) + t0;
  int16_t tf = t;
  if (tf > 511)
  {
    tf = tf - 1024;
  }
  float ta = tf * 0.004;
  return ta;
}
float gF_net(float gx, float gy, float gz)
{
  return sqrt(gx * gx + gy * gy + gz * gz);
}

float bubbleSort(float arr[], int num)
{
  int x, y;
  float Temp;
  for (x = 0; x < num - 1; x++)
  {
    for (y = 0; y < num - x - 1; y++)
    {
      if (arr[y] > arr[y + 1])
      {
        Temp = arr[y];
        arr[y] = arr[y + 1];
        arr[y + 1] = Temp;
      }
    }
  }
  return arr[num - 1];
}

void copy(float arr1[], float arr2[], int num)
{ // (F,f) copying F --> f
  for (int k = 0; k < num; k++)
  {
    arr2[k] = arr1[k];
  }
}

bool decision()
{ 
  ArduinoOTA.handle();
  copy(F, f, i);
  Serial.print(" F is sorted and  F max : ");
  Serial.println(bubbleSort(F, i));
  Serial.print(" index of F max : ");
  int index = index_finder(f, F[i - 1], i);
  Serial.println(index);

  if (index == (i - 1) || index == i)
  {
    if (index == (i - 1))
      Serial.println(" case 1");
    else
      Serial.println(" case 2");
    Serial.print(" Area upto F maximun : ");
    float A1 = Area(f, T, index);
    float T_sum = sum(T, index);
    A1 = A1 - (Ft * T_sum);
    Serial.println(A1);
    Serial.println("    values of T arry  : Avg / sum  ");
    float T_avg = avg(T, index);
    Serial.print(T_avg);
    Serial.print(" / ");
    Serial.println(T_sum);
    Serial.print("final output F avg : ");
    float F_avg = A1 / T_avg;
    Serial.println(F_avg);
    if (F_avg >= Fs)
    {
      return true;
    }
    else
      return false;
  }
  else
  {
    Serial.println(" case 3");
    Serial.println(" Area  :  A1 / A2 ");
    float A1 = Area(f, T, index);
    float T1 = sum(T, index);
    float Tt = sum(T, i - 1);
    float T2 = Tt - T1;
    A1 = A1 - (Ft * T1);
    Serial.print(A1);
    float A = Area(f, T, i - 1) - (Ft * Tt);
    float A2 = A - A1;
    Serial.print(" / ");
    Serial.println(A2);
    float F1_avg = A1 / T2;
    float F2_avg = A2 / T1;
    Serial.println("   F1_avg / F2_avg   ");
    Serial.print(F1_avg);
    Serial.print(" / ");
    Serial.println(F2_avg);
    if (F1_avg >= Fs || F2_avg >= Fs)
    {
      return true;
    }
    else
      return false;
  }
}

float avg(float arr[], int index)
{
  float sum = 0, avg = 0;
  for (int i = 0; i <= index; i++)
  {
    sum = sum + arr[i];
  }
  float n = (index + 1);
  avg = sum / n;
  return avg;
}
float sum(float arr[], int index)
{
  float sum = 0;
  for (int i = 0; i <= index; i++)
  {
    sum = sum + arr[i];
  }
  return sum;
}
float Area(float arr1[], float arr2[], int index)
{
  float area = 0;
  for (int j = 0; j <= index; j++)
  {
    area += arr1[j] * arr2[j];
  }
  return area;
}
int index_finder(float arr[], float val, int num)
{
  for (int k = 0; k < num; k++)
  {
    if (arr[k] == val)
    {
      return k;
    }
  }
  return 0;
}

float rT(char x)
{
  // runTime calculation function with error of +-1.00
  float a = 0, b = 0;
  if (x == 's')
  {
    //    Serial.println();
    //    Serial.println("             ---------------------  start  ---------------         ");
    a = millis();
    t = a;
    return a;
  }
  else if (x == 'e')
  {
    b = millis() - t;
    //        Serial.println();
    //        Serial.print("             ---------------------  runTime  : ");
    //        Serial.print(b);
    //        Serial.println("  ---------------------");
    return b;
  }
  return 0;
}

void ShowSerialData()
{
  while (Serial.available())
  {
    gprsSerial.write(Serial.read()); // Forward what Serial received to Software Serial Port
  }
  while (gprsSerial.available())
  {
    Serial.write(gprsSerial.read()); // Forward what Software Serial received to Serial Port
  }
}
void gprs_data_transfer(float Lat, float Long)
{
  ArduinoOTA.handle();
  gprsSerial.println("AT");
  delay(1000);
  ShowSerialData();
  gprsSerial.println("AT+CPIN?");
  delay(1000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CREG?");
  delay(1000);
  ShowSerialData();
  gprsSerial.println("AT+CGATT?");
  delay(1000);
  ShowSerialData();
  gprsSerial.println("AT+CIPSHUT");
  delay(1000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIPSTATUS");
  delay(1000);
  ShowSerialData();
  gprsSerial.println("AT+CIPMUX=0");
  delay(1000);
  ShowSerialData();
  gprsSerial.println("AT+CSTT=\"airtelgprs.com\""); // start task and setting the APN,
  delay(1000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIICR"); // bring up wireless connection
  delay(3000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIFSR"); // get local IP adress
  delay(2000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIPSPRT=0");
  delay(2000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\""); // start up the connection
  delay(2000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIPSEND"); // begin send data to remote server
  delay(3000);
  ArduinoOTA.handle();
  ShowSerialData();
  String str = "GET https://api.thingspeak.com/update?api_key=U7B4UYVVZP87A0E3&field1=" + Device_id + "&field2=" + String(Lat) + "&field3=" + String(Long); //+"&field4="+String(impact);
  Serial.println(str);
  gprsSerial.println(str); // begin send data to remote server
  delay(2000);
  ArduinoOTA.handle();
  delay(2000);
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println((char)26);// sending
  delay(2500);  // waitting for reply, important! the time is base on the condition of internet
  ArduinoOTA.handle();
  delay(2500);  
  ArduinoOTA.handle();
  ShowSerialData();
  gprsSerial.println("AT+CIPSHUT"); // close the connection
  delay(100);
  ShowSerialData();
  Serial.println("");
  Serial.println("-----------------Done-------------");
}

void gprs_handle(float Lat, float Long)
{
  while (true)
  {
    Serial.print("gprs_data_status : ");
    Serial.println(gprs_data_status);
    if (gprs_data_status == 0)
      gprs_data_transfer(Lat, Long);
    else
      break;
  }
}

bool gps_data_handle()
{
  ReadString = gps_serial.readStringUntil(13); // NMEA data ends with 'return' character, which is ascii(13)
  ReadString.trim();                           // they say NMEA data starts with "$", but the Arduino doesn't think so.
  // Serial.println(ReadString);         //All the raw sentences will be sent to monitor, if you want them, maybe to see the labels and data order.
  // Start Parsing by finding data, put it in a string of character array, then removing it, leaving the rest of thes sentence for the next 'find'
  if (ReadString.startsWith("$GPGLL"))
  {
    // I picked this sentence, you can pick any of the other labels and rearrange/add sections as needed.
    // mine looks like this: "$GPGLL,4053.16598,N,10458.93997,E,224431.00,A,D*7D"
    // This section gets repeated for each delimeted bit of data by looking for the commas
    // Find Lattitude is first in GLL sentence, other senetences have data in different order
    int Pos = ReadString.indexOf(','); // look for comma delimetrer
    ReadString.remove(0, Pos + 1);     // Remove Pos+1 characters starting at index=0, this one strips off "$GPGLL" in my sentence
    Pos = ReadString.indexOf(',');     // looks for next comma delimetrer, which is now the first comma because I removed the first segment
    char Lat[Pos];                     // declare character array Lat with a size of the dbit of data
    for (int i = 0; i <= Pos - 1; i++)
    { // load charcters into array
      Lat[i] = ReadString.charAt(i);
    }
    // Serial.print(Lat);          // display raw latitude data in Serial Monitor, I'll use Lat again in a few lines for converting
    // repeating with a different char array variable
    // Get Lattitude North or South
    ReadString.remove(0, Pos + 1);
    Pos = ReadString.indexOf(',');
    if (Pos == 0)
    {
      char LatSide[Pos + 1];
      LatSide[Pos] = char(0);
      // convert the variable array Lat to degrees Google can use
      float LatAsFloat = atof(Lat); // atof converts the char array to a float type
      if (LatSide[0] == char(78))
      {                                     // char(69) is decimal for the letter "N" in ascii chart
        LatInDeg = ConvertData(LatAsFloat); // call the conversion funcion (see below)
      }
      if (LatSide[0] == char(83))
      {                                        // char(69) is decimal for the letter "S" in ascii chart
        LatInDeg = -(ConvertData(LatAsFloat)); // call the conversion funcion (see below)
      }
      Latitude = LatInDeg;
    }
    else
    {
      char LatSide[Pos]; // declare different variable name
      for (int i = 0; i <= Pos - 1; i++)
      {
        LatSide[i] = ReadString.charAt(i); // fill the array
        // Serial.println(LatSide[i]);       //display N or S
      }
      // convert the variable array Lat to degrees Google can use
      float LatAsFloat = atof(Lat); // atof converts the char array to a float type
      if (LatSide[0] == char(78))
      {                                     // char(69) is decimal for the letter "N" in ascii chart
        LatInDeg = ConvertData(LatAsFloat); // call the conversion funcion (see below)
      }
      if (LatSide[0] == char(83))
      {                                        // char(69) is decimal for the letter "S" in ascii chart
        LatInDeg = -(ConvertData(LatAsFloat)); // call the conversion funcion (see below)
      }
      Latitude = LatInDeg;
    }
    // Serial.println(Latitude,15);
    // repeating with a different char array variable
    // Get Longitude
    ReadString.remove(0, Pos + 1);
    Pos = ReadString.indexOf(',');
    char Longit[Pos]; // declare different variable name
    for (int i = 0; i <= Pos - 1; i++)
    {
      Longit[i] = ReadString.charAt(i); // fill the array
    }

    //  Serial.print(Longit);      //display raw longitude data in Serial Monitor
    // repeating with a different char array variable
    // Get Longitude East or West
    ReadString.remove(0, Pos + 1);
    Pos = ReadString.indexOf(',');
    if (Pos == 0)
    {
      char LongitSide[Pos + 1]; // declare different variable name
      LongitSide[Pos] = char(0);
      // convert to degrees Google can use
      float LongitAsFloat = atof(Longit); // atof converts the char array to a float type
      if (LongitSide[0] == char(69))
      {                                         // char(69) is decimal for the letter "E" in ascii chart
        LongInDeg = ConvertData(LongitAsFloat); // call the conversion funcion (see below
      }
      if (LongitSide[0] == char(87))
      {                                            // char(87) is decimal for the letter "W" in ascii chart
        LongInDeg = -(ConvertData(LongitAsFloat)); // call the conversion funcion (see below
      }
      Longtitude = LongInDeg;
    }
    char LongitSide[Pos]; // declare different variable name
    for (int i = 0; i <= Pos - 1; i++)
    {
      LongitSide[i] = ReadString.charAt(i); // fill the array
      // Serial.println(LongitSide[i]);        //display raw longitude data in Serial Monitor
    }
    // convert to degrees Google can use
    float LongitAsFloat = atof(Longit); // atof converts the char array to a float type
    if (LongitSide[0] == char(69))
    {                                         // char(69) is decimal for the letter "E" in ascii chart
      LongInDeg = ConvertData(LongitAsFloat); // call the conversion funcion (see below
    }
    if (LongitSide[0] == char(87))
    {                                            // char(87) is decimal for the letter "W" in ascii chart
      LongInDeg = -(ConvertData(LongitAsFloat)); // call the conversion funcion (see below
    }
    Longtitude = LongInDeg;
    // repeating with a different char array variable
    // Get TimeStamp - GMT
    // ReadString.remove(0, Pos + 1);
    // Pos = ReadString.indexOf(',');
    // char TimeStamp[Pos]; // declare different variable name
    // for (int i = 0; i <= Pos - 1; i++)
    // {
    //   TimeStamp[i] = ReadString.charAt(i); // fill the array
    // }
    return true;
  }
  return false;
}

// Conversion function
float ConvertData(float RawDegrees)
{
  float RawAsFloat = RawDegrees;
  int firstdigits = ((int)RawAsFloat) / 100; // Get the first digits by turning f into an integer, then doing an integer divide by 100;
  float nexttwodigits = RawAsFloat - (float)(firstdigits * 100);
  float Converted = (float)(firstdigits + nexttwodigits / 60.0);
  return Converted;
}