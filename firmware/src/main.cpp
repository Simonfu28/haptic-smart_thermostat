#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <SimpleFOC.h>
#include <Adafruit_AHTX0.h>

#include <tlv_sensor.h>
#include <tlv_sensor.cpp>

#include <NotoSansBold15.h>
#include <NotoSansMedium24.h>
#include <NotoSansMedium50.h>


/********************** MQTT Communications Related **********************/
const char *mqtt_broker = "broker.emqx.io";   
const char *topic_in = "device/cloud_set_temp";
const char *topic_out = "device/device_sensor_temp";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;
// Wifi //
const char *ssid = "DESKTOP-8RINTUT 5950";
const char *password = "12345678";

WiFiClient espclient;
PubSubClient client(espclient);

void callback(char *topic, byte *payload, unsigned int length);
char t[50];

unsigned int lastMsg_time = 0;

void setup_wifi();
void setup_mqtt();

/************************* BLDC simpleFOC related *************************/
TlvSensor encoder = TlvSensor();
BLDCDriver6PWM driver = BLDCDriver6PWM(1, 2, 4, 5, 6, 7);
BLDCMotor motor = BLDCMotor(7);

#define MOTOR_VOLTAGE_SUPPLY 3.3
#define MOTOR_VOLTAGE_LIMIT 3.3
int setup_motor_control(float v_supply, float v_limit);

// haptic attraction controller //
PIDController P_haptic(3.5, 0, 0, 100000, MOTOR_VOLTAGE_LIMIT); // PIDController (P, I, D, ramp, voltage limit)
float attract_angle = 0;  // attractor angle variable
float attractor_distance = 15*(_PI/180.0); // attractor angle variable, dimp each X degrees
float findAttractor(float current_angle);
void motor_action();

// upper and lower angle and temperature limits for end stops //
double upper_anglelimit = 0;
double lower_anglelimit = 0;
#define UPPPER_TEMP_LIMIT 35
#define LOWER_TEMP_LIMIT 15

/*************************** TFT Screen Related ***************************/
#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_MEDIUM NotoSansMedium24
#define AA_FONT_LARGE NotoSansMedium50

#define TFT_MARKER_COLOR_RED 0xf44a
#define TFT_MARKER_COLOR_CYAN TFT_CYAN
#define TFT_BG_COLOR 0x4208

TFT_eSPI tft = TFT_eSPI();

void setup_tft();
double target_temp_marker(double target_temp, double prev_marker_theta);
double current_temp_marker(double current_temp, double prev_marker_theta);
void print_target_temp(double current_temp, int color);
double print_relative_humidity(double current_humidity, double previous_humidity, int color);

double target_temp_theta = 0;
double current_temp_theta = 1;

/************************** AHT20 Sensor Related **************************/
Adafruit_AHTX0 aht;
#define AHT_TEMP_OFFSET 3.2;
#define AHT_HUMIDITY_OFFSET 15.0;

double prev_humidity = 0;

/*************************** Multi Core Related ***************************/
void tft_task_code(void* pvParameters);
TaskHandle_t tft_task;

void motor_task_code(void* pvParameters);
TaskHandle_t motor_task;

void aht_sensor_update_code(void* pvParameters);
TaskHandle_t read_temperature_update;

void push_mqtt_data_code(void* pvParameters);
TaskHandle_t push_mqtt_data;

/**************************** Global Variables ****************************/
double temp_offset = 20; // starting temperature from motor position
double theta_offset; // temp_offset translated to motor position in radians
double current_temperature = 22;
double target_temperature = 20;
double current_humidity = 15;

/********************************* Setup *********************************/
void setup() {
  Serial.begin(115200);
  while (!Serial){ ; }

  setup_motor_control(MOTOR_VOLTAGE_SUPPLY, MOTOR_VOLTAGE_LIMIT);
  setup_tft();
  //setup_wifi();
  //setup_mqtt();
  aht.begin();
  
  xTaskCreatePinnedToCore (
    tft_task_code,
    "tft task",
    10000,
    NULL,
    5,
    &tft_task,
    0);

  xTaskCreatePinnedToCore (
    motor_task_code,
    "motor task",
    10000,
    NULL,
    5,
    &motor_task,
    1);
  
  xTaskCreatePinnedToCore (
    aht_sensor_update_code,
    "update display to current temp",
    10000,
    NULL,
    3,
    &read_temperature_update,
    0);

  /*xTaskCreate (
    push_mqtt_data_code,
    "push data values to mqtt broker",
    256,
    NULL,
    2,
    &push_mqtt_data);
    
    vTaskDelete(NULL);*/
  }

/********************************* Tasks **********************************/
void motor_task_code(void* pvParameters) {
  for(;;){
    motor.loopFOC();

    //Temperature based off absolute position of knob
    double temp = motor.shaft_angle - theta_offset;
    temp = temp_offset + 0.2*int(temp/attractor_distance);

    if (temp >= UPPPER_TEMP_LIMIT){
      if (upper_anglelimit == 0){
        upper_anglelimit = motor.shaft_angle;
      }
      motor.move(P_haptic(upper_anglelimit - motor.shaft_angle));
    }
    else if (temp <= LOWER_TEMP_LIMIT){
      if (lower_anglelimit == 0){
        lower_anglelimit = motor.shaft_angle;
      }
      motor.move(P_haptic(lower_anglelimit - motor.shaft_angle));
    }
    else {
      motor_action();
    }

    target_temperature = temp;
  }
}

void tft_task_code(void* pvParameters) {
  for(;;) {
    current_temp_theta = current_temp_marker(current_temperature, current_temp_theta);
    target_temp_theta = target_temp_marker(target_temperature, target_temp_theta);
    
    vTaskDelay(33/portTICK_PERIOD_MS);
  }
}

void aht_sensor_update_code(void* pvParameters) {
  for(;;) {
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp)){
      current_temperature = temp.temperature;
      current_humidity = humidity.relative_humidity;
    }
    else {
      Serial.println("AHT20 read error");
    }
    
    prev_humidity = print_relative_humidity(current_humidity, prev_humidity, TFT_MARKER_COLOR_CYAN);

    vTaskDelay(10000/portTICK_RATE_MS);
  }
}

void push_mqtt_data_code(void* pvParameters) {
  if (!client.connected()){
    client.connect("esp32_device", mqtt_username, mqtt_password);
  }
  else {
    client.loop();
    String temp_str = String(current_temperature);
    temp_str.toCharArray(t, temp_str.length() + 1);
    client.publish(topic_out, t);
  }

  vTaskDelay(15000/portTICK_RATE_MS);
}

void loop() {}

/******************************* Functions ********************************/
int setup_motor_control(float v_supply, float v_limit) {
  /********* SimpleFOC setup *********/
  /********* encoder setup *********/
  encoder.init(&Wire, false);
  // error check for encoder initialisation //
  /*if (encoder.init() != 0) {
    Serial.println("magnetic encoder init fail");
    Serial.println(encoder.init());
    return 1;
  }*/

  /********* driver setup *********/
  driver.voltage_power_supply = v_supply;
  driver.voltage_limit = v_limit;
  // error check for driver initialisation //
  if (driver.init() != 1) {
    Serial.println("driver init fail");
    Serial.println(driver.init());
    return 2;
  }

  /********* motor setup *********/
  motor.linkSensor(&encoder);
  motor.linkDriver(&driver);
  // torque control loop //
  motor.controller = MotionControlType::torque;  
  motor.torque_controller = TorqueControlType::voltage;
  motor.init();

  if (motor.initFOC() != 1) {
    Serial.println("motor init fail");
    Serial.println(motor.initFOC());
    return 3;
  }

  theta_offset = motor.shaft_angle;
  return 0;
}

float findAttractor(float current_angle){
  /********* finds closest detent *********/
  return round(current_angle/attractor_distance)*attractor_distance;
}

void motor_action(){
  /********* motor haptic movement *********/
  // Motion control function
  motor.move(P_haptic(attract_angle - motor.shaft_angle));
  // calculate the attractor
  attract_angle = findAttractor(motor.shaft_angle);
}

double target_temp_marker(double target_temp, double prev_marker_theta) {
  /********* prints marker for the set target temperature *********/
  // arc from 30 to 330, temp range from 5 to 45
  // previous polars for marker
  int R = 105;
  int x = 120 + R*cos(prev_marker_theta);
  int y = 120 + R*sin(prev_marker_theta);
  double prev_temp = (prev_marker_theta - (PI/2))/(7.5*0.0174533);
  // if marker has moved, erase previous marker //
  if (prev_marker_theta != (7.5*target_temp)*0.0174533 + PI/2){
    tft.fillSmoothCircle(x, y, 6, TFT_BG_COLOR, TFT_BG_COLOR);
    print_target_temp(prev_temp, TFT_BG_COLOR);
  }

  // new polars for marker
  prev_marker_theta = (7.5*target_temp)*0.0174533 + PI/2;
  x = 120 + R*cos(prev_marker_theta);
  y = 120 + R*sin(prev_marker_theta);
  tft.fillSmoothCircle(x, y, 6, TFT_MARKER_COLOR_RED, TFT_MARKER_COLOR_RED);
  print_target_temp(target_temp, TFT_MARKER_COLOR_RED);
  
  return prev_marker_theta;
}

double current_temp_marker(double current_temp, double prev_marker_theta) {
  /********* prints the marker and text for the current temperature *********/
  // arc from 30 to 330, temp range from 5 to 45
  // previous polars for marker
  int R_circle = 105;
  int x_circle = 120 + R_circle*cos(prev_marker_theta);
  int y_circle = 120 + R_circle*sin(prev_marker_theta);

  tft.loadFont(AA_FONT_SMALL);
  tft.setTextDatum(MC_DATUM);

  // previous polars for text
  int R_text = 80;
  int x_text = 120 + R_text*cos(prev_marker_theta);
  int y_text = 120 + R_text*sin(prev_marker_theta);
  // if marker has moved, erase previous marker and text //
  if (prev_marker_theta != (7.5*current_temp)*0.0174533 + PI/2){
    tft.fillSmoothCircle(x_circle, y_circle, 6, TFT_BG_COLOR, TFT_BG_COLOR);
    tft.setTextColor(TFT_BG_COLOR, TFT_BG_COLOR, true);
    double prev_temp = (prev_marker_theta - PI/2)/(7.5*0174533);
    String out_str = String(current_temp, 1) + "°";
    tft.drawString(out_str, x_text, y_text);
  }

  // new polars for marker and text
  prev_marker_theta = (7.5*current_temp)*0.0174533 + PI/2;
  x_circle = 120 + R_circle*cos(prev_marker_theta);
  y_circle = 120 + R_circle*sin(prev_marker_theta);
  x_text = 120 + R_text*cos(prev_marker_theta);
  y_text = 120 + R_text*sin(prev_marker_theta);

  tft.setTextColor(TFT_MARKER_COLOR_CYAN, TFT_BG_COLOR, true);
  tft.fillSmoothCircle(x_circle, y_circle, 6, TFT_MARKER_COLOR_CYAN, TFT_MARKER_COLOR_CYAN);
  String out_str = String(current_temp, 1) + "°";
  tft.drawString(out_str, x_text, y_text);
  tft.unloadFont();

  return prev_marker_theta;
}

void print_target_temp(double current_temp, int color) {
  /********* prints large set temeprature in middle of screen *********/
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(color, TFT_BG_COLOR, true);

  tft.loadFont(AA_FONT_LARGE);
  String out_str = String(current_temp, 1) + "°";
  tft.drawString(out_str, 120, 125);
  tft.unloadFont();
}

double print_relative_humidity(double current_humidity, double previous_humidity, int color) {
  /********* prints RH text in middle of screen *********/
  tft.setTextDatum(MC_DATUM);
  tft.loadFont(AA_FONT_SMALL);
  int position_x = 120;
  int position_y = 165;
  String out_str = "RH: " + String(previous_humidity, 1) + "%";

  // erase previous humidity //
  if (previous_humidity == current_humidity) {
    tft.setTextColor(TFT_BG_COLOR, TFT_BG_COLOR, true);
    tft.drawString(String(previous_humidity, 1), position_x, position_y);
  }

  // print and return current humidity //
  tft.setTextColor(color, TFT_BG_COLOR, true);
  out_str = ("RH: " + String(current_humidity, 1) + "%");
  tft.drawString(out_str, position_x, position_y);
  tft.unloadFont();

  return current_humidity;
}

void setup_tft() {
  /********* tft screen setup *********/
  tft.init();
  tft.fillScreen(TFT_BG_COLOR);
  
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_MARKER_COLOR_RED, TFT_BG_COLOR, true);

  tft.loadFont(AA_FONT_MEDIUM);
  tft.drawString("Set", 120, 80);
  tft.unloadFont();
}

void setup_wifi() {
  /********* ESP32 wifi setup *********/
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFI Connecting.......Status: ");
    Serial.println(WiFi.status());
    delay(2000);
    //WiFi.begin(ssid, password);
  }
  Serial.println("WiFi Connected");
}

void setup_mqtt() {
  /********* PubSubClient MQTT setup *********/
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()){
    Serial.println("connecting to mqtt broker");
    if (client.connect("esp32_device", mqtt_username, mqtt_password)){
      Serial.println("connected to mqtt broker!");
    }
    else {
      Serial.print("failed with state ");
      Serial.println(client.state());
    }
    delay(2000);
  }
  client.subscribe(topic_in);
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic_in);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}
