#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Adafruit_ADS1X15.h>
const int Analog_channel_pin= 33;
int ADC_VALUE = 0;
float voltage_value = 0; 
float current = 0;

unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 10000;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 18000);
String datestamp,timestamp;

#define MAX_DATA_SIZE 21

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_SSID "Extensity" //TP-LINK_B4EA //Extensity //26585-Multi-03233414511 //Connectify-me //
#define WIFI_PASSWORD "password1" //55673662 //password1 //Najeeb4455 // 11223344

#define API_KEY "AIzaSyA1mw9AntbhRry4tXVmK_Vae-bPAeVFnfA"
#define DATABASE_URL "https://turbine-8280c-default-rtdb.firebaseio.com/" 

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

byte identity, security;
int windDirection, temperature, humidity, windSpeed;
int gustSpeed, rainfall, UV, light, barometricPressure;
int n=1;

byte thirdByte, fourthByteFirstNibble;

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  
  Serial.println("Getting single-ended readings from AIN0..3");
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  if (!ads.begin()) {Serial.println("Failed to initialize ADS.");while (1);}
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {Serial.print(".");delay(300);}
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {Serial.println("ok");signupOK = true;}
  else {Serial.printf("%s\n", config.signer.signupError.message.c_str());}
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

timeClient.begin();
printLocalTime();
}

void loop() { 
  unsigned long currentMillis = millis();
  static byte data[MAX_DATA_SIZE];
  static int dataIndex = 0;

  if (Serial2.available()) {
    while (Serial2.available()) {
      char c = Serial2.read();
      String hexData = String(c, HEX);
      data[dataIndex] = strtol(hexData.c_str(), NULL, 16);
      dataIndex++;

      if (dataIndex >= MAX_DATA_SIZE) {
        processData(data, dataIndex);
        dataIndex = 0;
        memset(data, 0, sizeof(data));
      }
    }
    Serial.println();
  }
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    currentTurbine();
    voltageTurbine();
    printLocalTime();
    setfirebase(datestamp,timestamp);
  }
}

void printLocalTime()
{
  timeClient.forceUpdate();
  unsigned long epochTime = timeClient.getEpochTime();
  time_t now = static_cast<time_t>(epochTime);
  struct tm * timeInfo = localtime(&now);
  int day,month,year,hour,minute,second;

  day=timeInfo->tm_mday;
  month=timeInfo->tm_mon; // tm_mon is 0-based (0-11)
  year=timeInfo->tm_year+1900; // tm_year is years since 1900
  datestamp=String(day)+":"+String(month)+":"+String(year);
  Serial.println("Date: " + datestamp);

  hour=timeInfo->tm_hour;
  minute=timeInfo->tm_min;
  second=timeInfo->tm_sec;
  timestamp=String(hour)+":"+String(minute)+":"+String(second);
  Serial.println("Time: " + timestamp);
}

void currentTurbine()
{
  int16_t adc0;
  float volts0;

  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);
  current = (volts0 - 2.4) / 0.066;

  if (current < 0.1) {current = 0;}
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  Serial.print("current : ");Serial.println(current);
}

void voltageTurbine()
{
  delay(100);
  ADC_VALUE = analogRead(Analog_channel_pin);
  delay(300);
  Serial.print("ADC VALUE = ");Serial.println(ADC_VALUE);
  delay(100);
  voltage_value = (ADC_VALUE * 24.0 ) / (4095);
  Serial.print("Voltage = ");Serial.print(voltage_value);Serial.println(" volts ");
  delay(100);
}

void processData(byte data[], int size) {
  identity = data[1-n];
  security = data[2-n];
  thirdByte = data[3-n];
  fourthByteFirstNibble = data[4-n] >> 4;
  windDirection = (fourthByteFirstNibble << 8) | thirdByte;
  byte fourthByteSecondNibble = (data[4-n] & 0x0F);
  byte fifthByte = data[5-n];
  temperature = (fourthByteSecondNibble << 8) | fifthByte;
  humidity = data[6-n];
  windSpeed = data[7-n];
  gustSpeed = data[8-n];
  byte rainfall1 = data[9-n];
  byte rainfall2 = data[10-n];
  rainfall = (rainfall1 << 8) | rainfall2;
  byte UV1 = data[11-n];
  byte UV2 = data[12-n];
  UV = (UV1 << 8) | UV2;
  byte light1 = data[13-n];
  byte light2 = data[14-n];
  byte light3 = data[15-n];  
  light = light1 << 16 | light2 << 8 | light3;
  byte barometricPressure1 = data[18-n];
  byte barometricPressure2 = data[19-n];
  byte barometricPressure3 = data[20-n]; 
  barometricPressure = barometricPressure1 << 16 | barometricPressure2 << 8 | barometricPressure3;
  
  
//  for (int i = 0; i < size; i++) {Serial.print(data[i], BIN);Serial.print(' ');}Serial.println();  
//  for (int i = 0; i < size; i++) {Serial.print(data[i], HEX);Serial.print(' ');}Serial.println();
//  Serial.print("id: " + String(identity,HEX) + ", sec: " + String(security,HEX) + ", windD: " + String(windDirection,HEX));
//  Serial.print(", temp: " + String(temperature,HEX) + ", hum: " + String(humidity,HEX) + ", windS: " + String(windSpeed,HEX));
//  Serial.println(", gSpeed: " + String(gustSpeed,HEX) + ", rain: " + String(rainfall,HEX) + ", UV: " + String(UV,HEX));
//  Serial.println(", light: " + String(light,HEX) + ", barPressure: " + String(barometricPressure,HEX));
//  Serial.println();
}

void setfirebase(String datestamp, String timestamp)
{
    //Callibrated Weather Station Parameters
    if (fourthByteFirstNibble & B1000) {fourthByteFirstNibble = B0001;}  //if the MSB is 1 than 1 is stored
    else {fourthByteFirstNibble = B0000;}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/windDirection", windDirection)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/temperature", (temperature-400)/10)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/humidity", humidity)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/windSpeed", (windSpeed/8)*0.51)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/gustSpeed", gustSpeed*0.51)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/rainfall", rainfall*0.254)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/UV", UV)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/light", light/10.0)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/weather/barometricPressure", barometricPressure/100.0)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}

    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/voltage", voltage_value)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
    if (Firebase.RTDB.setFloat(&fbdo, datestamp+"/"+timestamp+"/current", current)){Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());Serial.println("TYPE: " + fbdo.dataType());}
    else {Serial.println("FAILED");Serial.println("REASON: " + fbdo.errorReason());}
}
