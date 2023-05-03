#include <Arduino.h>
#include "SPI.h"
#include "TFT_eSPI.h"

#include "FS.h"
#include "SPIFFS.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

const char* ssid = "";                  
const char* password = "";              

#define LCD_LED_GPIO   17
#define PWM1_Ch    0
#define PWM1_Res   8
#define PWM1_Freq  1000

#define WHEEL_SENSOR 27
#define PEDAL_SENSOR 26

#define CALIBRATION_FILE    "/TouchCalData1"
#define RECALIBRATE_TOUCH   false
#define TOUCH_THRESHOLD     300

#define ACCEL_OFFSET_X    -0.64
#define ACCEL_OFFSET_Y    -0.24
#define ACCEL_OFFSET_Z    -1.46

int PWM1_DutyCycle = 0;

static uint32_t last_wheel = 1;
static uint32_t last_last_wheel = 0;
static uint32_t last_five_wheel[5] = {0, 1, 2, 3, 4};

static uint32_t last_pedal = 1;
static uint32_t last_last_pedal = 0;
static uint32_t last_eight_pedal[8] = {0, 1, 2, 3, 4, 5, 6, 7};

static float pitches[100] = {0};

static uint16_t logger_cadence = 0;
static uint16_t logger_speed = 0;
static uint16_t logger_gear = 0;
String logger_buffer = "";

hw_timer_t *logger_timer = NULL;
fs::File file;

uint16_t x, y;
uint16_t ota_counter = 0;

TFT_eSPI tft = TFT_eSPI();
Adafruit_MPU6050 mpu;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
AsyncWebServer server(80);

uint32_t get_mph();
uint32_t get_cadence();

void pedal_interrupt() {

  //if less than 20ms since last interrupt, ignore it
  if (millis() - last_pedal < 20) return;

  logger_cadence += 1;

  last_last_pedal = last_pedal;
  last_pedal = millis(); 

  //shuffle along the array
  for (int i = 0; i < 7; i++) {
    last_eight_pedal[i] = last_eight_pedal[i+1];
  }
  last_eight_pedal[7] = last_pedal; 

}

void wheel_interrupt() {

  //if less than 20ms since last interrupt, ignore it
  if (millis() - last_wheel < 100) return;

  logger_speed += 1;

  last_last_wheel = last_wheel;
  last_wheel = millis();

  //shuffle along the array
  for (int i = 0; i < 4; i++) {
    last_five_wheel[i] = last_five_wheel[i+1];
  }
  last_five_wheel[4] = last_wheel;

}

void IRAM_ATTR logger_timer_callback() {
  // Write last values to file system
  logger_buffer += millis();
  logger_buffer += ",";

  logger_buffer += logger_cadence;
  logger_buffer += ",";

  logger_buffer += logger_speed;
  logger_buffer += ",";

  logger_buffer += logger_gear;
  logger_buffer += ",";

  // Zero values
  logger_cadence = 0;
  logger_speed = 0;
}

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (RECALIBRATE_TOUCH)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      fs::File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !RECALIBRATE_TOUCH) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println(F("Touch corners as indicated"));

    tft.setTextFont(1);
    tft.println();

    if (RECALIBRATE_TOUCH) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println(F("Set RECALIBRATE_TOUCH to false to stop this running again!"));
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println(F("Calibration complete!"));

    // store data
    fs::File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }

  }
}

//Basically a faster drop in replacement for tft.getTouch()
//Also accounts for rotation properly
//wait_z_stop makes it a fair bit slower, its for single touches not swiping
bool get_touch(uint16_t *x, uint16_t *y, bool wait_z_stop) {
  uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

  if (tft.getTouchRawZ() < TOUCH_THRESHOLD) return false;
  tft.getTouchRaw(&x_tmp, &y_tmp);

  if (wait_z_stop) {
    uint16_t oldz = tft.getTouchRawZ();
    delayMicroseconds(100);
    uint16_t newz = tft.getTouchRawZ();
    while (newz > oldz) {
      oldz = newz;
      newz = tft.getTouchRawZ();
      delayMicroseconds(100);
    }
  }

  delayMicroseconds(100);

  if (tft.getTouchRawZ() < TOUCH_THRESHOLD) return false;
  tft.getTouchRaw(&x_tmp2, &y_tmp2);

  if (abs(x_tmp2 - x_tmp) > 30) return false;
  if (abs(y_tmp2 - y_tmp) > 30) return false;

  tft.convertRawXY(&x_tmp, &y_tmp);

  float screen_width = tft.getViewportWidth();
  float screen_height = tft.getViewportHeight();

  switch (tft.getRotation()) {
    case 0: {
      *x = x_tmp;
      *y = y_tmp;
      break;
    }
    case 1: {
      *x = y_tmp * (screen_width/screen_height);
      *y = screen_height - (x_tmp * (screen_height/screen_width));
      break;
    }
    case 2: {
      *x = screen_width - x_tmp;
      *y = screen_height - y_tmp;
      break;
    }
    case 3: {
      *x = screen_width - (y_tmp * (screen_width/screen_height));
      *y = x_tmp * (screen_height/screen_width);
      break;
    }
  }

  return true;
}

uint32_t averaged_pedal_speed() {
  int total = 0;
  for (int i = 0; i < 7; i++) {
    total += last_eight_pedal[i+1] - last_eight_pedal[i];
  }

  //convert to rpm
  return (total / 7);
}

uint32_t averaged_wheel_speed() {
  int total = 0;
  for (int i = 0; i < 4; i++) {
    total += last_five_wheel[i+1] - last_five_wheel[i];
  }
  
  //convert to rpm
  return (total / 4);
}

uint32_t get_mph() {
  uint32_t rpm;

  if (millis() - last_wheel > ((last_wheel - last_last_wheel) * 2)) {
    rpm = 60000/(millis() - last_wheel);
  }
  else {
    rpm = 60000/(last_wheel - last_last_wheel);
  }

  return (0.677*PI*rpm*2.23694)/60;
}

//There are 8 magnets per rotation for cadence
uint32_t get_cadence() {

  if (millis() - last_pedal > ((last_pedal - last_last_pedal)*2)) {
    return 60000/((millis() - last_pedal)*8);
  }
  else {
    return 60000/((averaged_pedal_speed())*8);
  }

}



//-------------------------------Drawing Funcs-------------------------------//
#define TEXT_FG TFT_WHITE
#define TEXT_BG TFT_BLACK
#define BG TFT_BLACK
#define DARK_SHIFT_INDICATORS TFT_DARKGREY
#define LIGHT_SHIFT_INDICATORS TFT_LIGHTGREY


void draw_main() {
  // Cls and reset
  tft.fillScreen(BG);
  tft.setCursor(5, 2);
  tft.setTextColor(TEXT_FG, TEXT_BG);
  tft.setTextSize(2);

  // Divider line between top bar and main screen
  tft.drawFastHLine(3, 20, 314, TEXT_FG);

  // Draw boxes for speed and cadence
  tft.drawRect(5, 30, 122, 70, TEXT_FG);
  tft.drawRect(5, 110, 122, 70, TEXT_FG);
  
  // Draw labels for speed and cadence
  tft.setTextDatum(BL_DATUM);
  tft.setTextSize(3);
  tft.drawString("MPH", 135, 98);
  tft.drawString("RPM", 135, 178);

  // Draw dark grey shift up/down and circle indicators
  //up arrow
  tft.fillTriangle(200, 64, 245, 64, 223, 30, DARK_SHIFT_INDICATORS);
  tft.fillRect(213, 64, 19, 90-64, DARK_SHIFT_INDICATORS);

  //center circle
  tft.fillRoundRect(213, 96, 19, 19, 9, DARK_SHIFT_INDICATORS);

  //down arrow
  tft.fillTriangle(200, 146, 245, 146, 223, 180, DARK_SHIFT_INDICATORS);
  tft.fillRect(213, 120, 19, 180-146, DARK_SHIFT_INDICATORS);

}

void draw_main_dynamic() {
  tft.setCursor(5, 2);
  tft.setTextColor(TEXT_FG, TEXT_BG);
  tft.setTextSize(2);

  // Draw time
  int hour = timeClient.getHours();
  int minute = timeClient.getMinutes();
  int second = timeClient.getSeconds();

  tft.setTextSize(2);
  if (hour < 10) { tft.print("0"); }
  tft.print(timeClient.getHours());
  tft.print(":");
  if (minute < 10) { tft.print("0"); }
  tft.print(timeClient.getMinutes());
  tft.print(":");
  if (second < 10) { tft.print("0"); }
  tft.print(timeClient.getSeconds());
  tft.print(" ");

  // Draw date
  String formattedDate;
  tft.setTextSize(2);
  formattedDate = timeClient.getFormattedDate();
  tft.setTextDatum(TR_DATUM);
  tft.drawString(formattedDate.substring(0, 10), 315, 2);

  // Draw speed and cadence
  tft.setTextSize(6);
  tft.setTextDatum(TR_DATUM);

  uint32_t speed = get_mph();
  uint32_t cadence = get_cadence();
  tft.drawNumber(speed, 120, 44);
  tft.drawNumber(cadence, 120, 124);

  // Text padding doesnt work for rtl so we have to do it manually
  if (speed < 100) {
    tft.fillRect(6, 31, 40, 68, TEXT_BG);
  }
  if (speed < 10) {
    tft.fillRect(46, 31, 38, 68, TEXT_BG);
  }

  if (cadence < 100) {
    tft.fillRect(6, 111, 40, 68, TEXT_BG);
  }
  if (cadence < 10) {
    tft.fillRect(46, 111, 38, 68, TEXT_BG);
  }
  

  float average_pitch = 0;
  for (int i = 0; i < 99; i++) {
    average_pitch += pitches[i];
  }
  average_pitch = average_pitch / 100;

  float scaled_pitch = average_pitch * 10;
  if (scaled_pitch > 80) { scaled_pitch = 80; }
  if (scaled_pitch < -80) { scaled_pitch = -80; }
  if (scaled_pitch > -2 && scaled_pitch < 2) { scaled_pitch = 0; }
  // center y is 105
  if (scaled_pitch > 0) {
    tft.fillRect(270, 105, 35, scaled_pitch, TFT_RED);
    tft.fillRect(270, 106 + scaled_pitch, 35, 81-scaled_pitch, BG);
    tft.fillRect(270, 25, 35, 80, BG);

  }
  else {
    tft.fillRect(270, 105 - abs(scaled_pitch), 35, abs(scaled_pitch), TFT_GREEN);
    tft.fillRect(270, 105-80, 35, 80-abs(scaled_pitch), BG);
    tft.fillRect(270, 105, 35, 80, BG);
  }
  if (scaled_pitch == 0) { tft.fillRect(270, 105-2, 25, 5, BG); }


  static float optimal_cadence;
  optimal_cadence = 70;
  optimal_cadence += average_pitch * 1.5;
  // Calculate if the cyclist should shift up or down
  if (cadence > 10) {  // If actually cycling
    
    if (cadence > optimal_cadence + 10) {  // If cadence is too high, shift up
      tft.fillTriangle(200, 64, 245, 64, 223, 30, LIGHT_SHIFT_INDICATORS);
      tft.fillRect(213, 64, 19, 90-64, LIGHT_SHIFT_INDICATORS);

      tft.fillRoundRect(213, 96, 19, 19, 9, DARK_SHIFT_INDICATORS);

      tft.fillTriangle(200, 146, 245, 146, 223, 180, DARK_SHIFT_INDICATORS);
      tft.fillRect(213, 120, 19, 180-146, DARK_SHIFT_INDICATORS);

    }

    else if (cadence < optimal_cadence - 10) {  // If cadence is too low, shift down
      tft.fillTriangle(200, 146, 245, 146, 223, 180, LIGHT_SHIFT_INDICATORS);
      tft.fillRect(213, 120, 19, 180-146, LIGHT_SHIFT_INDICATORS);

      tft.fillRoundRect(213, 96, 19, 19, 9, DARK_SHIFT_INDICATORS);

      tft.fillTriangle(200, 64, 245, 64, 223, 30, DARK_SHIFT_INDICATORS);
      tft.fillRect(213, 64, 19, 90-64, DARK_SHIFT_INDICATORS);
    }

    else {  // If cadence is optimal
      tft.fillRoundRect(213, 96, 19, 19, 9, LIGHT_SHIFT_INDICATORS);

      tft.fillTriangle(200, 64, 245, 64, 223, 30, DARK_SHIFT_INDICATORS);
      tft.fillRect(213, 64, 19, 90-64, DARK_SHIFT_INDICATORS);

      tft.fillTriangle(200, 146, 245, 146, 223, 180, DARK_SHIFT_INDICATORS);
      tft.fillRect(213, 120, 19, 180-146, DARK_SHIFT_INDICATORS);
    }

  }
  else {              // If not cycling light up centre dot
    tft.fillRoundRect(213, 96, 19, 19, 9, LIGHT_SHIFT_INDICATORS);

    tft.fillTriangle(200, 64, 245, 64, 223, 30, DARK_SHIFT_INDICATORS);
    tft.fillRect(213, 64, 19, 90-64, DARK_SHIFT_INDICATORS);

    tft.fillTriangle(200, 146, 245, 146, 223, 180, DARK_SHIFT_INDICATORS);
    tft.fillRect(213, 120, 19, 180-146, DARK_SHIFT_INDICATORS);
  }

  tft.setCursor(120, 0);
  tft.setTextSize(2);
  // tft.print(average_pitch);
  tft.print(optimal_cadence);


}

void setup() {
  // This lets us measure the boot time
  tft.init();
  tft.fillScreen(TFT_WHITE);

  // Init LCD backlight
  ledcAttachPin(LCD_LED_GPIO, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcWrite(PWM1_Ch, 255);

  // Init serial
  Serial.begin(9600);
  while (!Serial);

  // Init SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("SPIFFS Failed");
  }

  // Init Touch
  touch_calibrate();

  // Init Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Continuing without wifi...");
    // delay(2000);
    // ESP.restart();
  }

  if (WiFi.isConnected()) {

    // Init OTA
    ArduinoOTA
      .onStart([]() {
        // timerAlarmDisable(logger_timer);
        // timerEnd(logger_timer);
        // timerAlarmWrite(logger_timer, 0, false);

        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Quick update check
    for (int i = 0; i < 200; i++) {
      ArduinoOTA.handle();
      delay(10);
    }

    // Init NTP
    timeClient.begin();
    timeClient.setTimeOffset(3600);
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }

    // Init Webserial
    WebSerial.begin(&server);
    server.begin();

  }


  // Init sensors pins
  pinMode(PEDAL_SENSOR, INPUT);
  pinMode(WHEEL_SENSOR, INPUT);

  // Init sensors interrupts
  attachInterrupt(PEDAL_SENSOR, pedal_interrupt, RISING);
  attachInterrupt(WHEEL_SENSOR, wheel_interrupt, RISING);

  // Init MPU
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);


  // Finish setting up LCD
  tft.setRotation(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);

  draw_main();

  
  // file = SPIFFS.open("/test.txt", FILE_WRITE, true);

  // logger_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(logger_timer, &logger_timer_callback, true);
  // timerAlarmWrite(logger_timer, 400000, true); // Trigger every second
  // timerAlarmEnable(logger_timer); //Just Enable
}


static uint32_t last_click = 0;
static uint32_t last_screen_update = 0;

void loop() {

  draw_main_dynamic();

 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch = (atan2((a.acceleration.y + ACCEL_OFFSET_Y), (a.acceleration.z + ACCEL_OFFSET_Z)) * 57.3) - 33.14;
  for (int i = 0; i < 99; i++) {
    pitches[i] = pitches[i+1];
  }
  pitches[99] = pitch;

  // The following commented code is for the logger
  // if (get_touch(&x, &y, true)) {

  //   if (last_click + 1000 < millis()) {
  //     last_click = millis();

  //     if (x > 200 && x < 245 && y > 30 && y < 90) {
  //       if (logger_gear < 9) logger_gear++;
  //     }
  //     if (x > 200 && x < 245 && y > 120 && y < 180) {
  //       if (logger_gear > 0) logger_gear--;
  //     }

  //     if (x > 270 && x < 312 && y > 36 && y < 90) {
  //       // print entire contents of file
  //       timerAlarmDisable(logger_timer);

  //       WebSerial.print("closing file...");
  //       file.close();
  //       file = SPIFFS.open("/test.txt", FILE_READ);
  //       while(file.available()){
  //         WebSerial.println(file.readStringUntil('\n'));
  //       }
  //       file.close();

  //       file = SPIFFS.open("/test.txt", FILE_WRITE, true);
  //       timerAlarmEnable(logger_timer);
  //     }

  //   }
  // }

  
  // tft.drawNumber(logger_gear, 170, 90);

  // if (logger_buffer != "") {
  //   sensors_event_t a, g, temp;
  //   mpu.getEvent(&a, &g, &temp);
    
  //   logger_buffer += String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z);
  //   logger_buffer += "," + String(get_mph()) + "," + String(get_cadence());
    
  //   file.println(logger_buffer);
  //   logger_buffer = "";
  // }

  // ota_counter++;
  // if (ota_counter > 10 && logger_gear == 0) {
  //   ota_counter = 0;

  //   timerAlarmDisable(logger_timer);
  //   ArduinoOTA.handle();
  //   timerAlarmEnable(logger_timer);
  // }

  delay(10);
  yield();
  ArduinoOTA.handle();

}