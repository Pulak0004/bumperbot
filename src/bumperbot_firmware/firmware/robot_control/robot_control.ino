#include <PID_v1.h>

#define left_enb 9
#define left_in1 11
#define left_in2 12
#define left_encoder_a 3
#define left_encoder_b 5

#define right_enb 14
#define right_in1 15
#define right_in2 16
#define right_encoder_a 4
#define right_encoder_b 6

unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;

String right_encoder_sign = "p";
String left_encoder_sign = "p";

double left_wheel_measured_vel = 0.0;
double right_wheel_measured_vel = 0.0;
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;
double right_wheel_cmd = 0.0;
double left_wheel_cmd = 0.0;

double kp_r = 11.5;
double ki_r = 7.5;
double kd_r = 0.1;
double kp_l = 12.8;
double ki_l = 8.3;
double kd_l = 0.1;

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_cmd_complete = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;

char value[] = "00.00";
char value_idx = 0;

PID rightMotor(&right_wheel_measured_vel, &right_wheel_cmd, &right_wheel_cmd_vel, kp_r, ki_r, kd_r, DIRECT);
PID leftMotor(&left_wheel_measured_vel, &left_wheel_cmd, &left_wheel_cmd_vel, kp_l, ki_l, kd_l, DIRECT);


void setup() {
  pinMode(left_enb, OUTPUT);
  pinMode(left_in1, OUTPUT);
  pinMode(left_in2, OUTPUT);
  pinMode(left_encoder_a, INPUT);
  pinMode(left_encoder_b, INPUT);

  pinMode(right_enb, OUTPUT);
  pinMode(right_in1, OUTPUT);
  pinMode(right_in2, OUTPUT);
  pinMode(right_encoder_a, INPUT);
  pinMode(right_encoder_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(left_encoder_a), leftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_a), rightEncoderCallback, RISING);

  digitalWrite(left_in1, HIGH);
  digitalWrite(left_in2, LOW);
  digitalWrite(right_in1, HIGH);
  digitalWrite(right_in2, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}


void loop() {
  if (Serial.available()) 
  {
    char chr = Serial.read();
    if (chr == "r") 
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    } 
    else if (chr == "l") 
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    } 
    else if (chr == "p") 
    {
      if (is_right_wheel_cmd && !is_right_wheel_forward) 
      {
        digitalWrite(right_in1, HIGH - digitalRead(right_in1));
        digitalWrite(right_in2, HIGH - digitalRead(right_in2));
        is_right_wheel_forward = true;
      } 
      else if (is_left_wheel_cmd && !is_left_wheel_forward) 
      {
        digitalWrite(left_in1, HIGH - digitalRead(left_in1));
        digitalWrite(left_in2, HIGH - digitalRead(left_in2));
        is_left_wheel_forward = true;
      }
    } 
    else if (chr == "n") 
    {
      if (is_right_wheel_cmd && is_right_wheel_forward) 
      {
        digitalWrite(right_in1, HIGH - digitalRead(right_in1));
        digitalWrite(right_in2, HIGH - digitalRead(right_in2));
        is_right_wheel_forward = false;
      } 
      else if (is_left_wheel_cmd && is_left_wheel_forward) 
      {
        digitalWrite(left_in1, HIGH - digitalRead(left_in1));
        digitalWrite(left_in2, HIGH - digitalRead(left_in2));
        is_left_wheel_forward = false;
      }
    } 
    else if (chr == ",") 
    {
      if (is_right_wheel_cmd) 
      {
        right_wheel_cmd_vel = atof(value);
      } 
      else if (is_left_wheel_cmd) 
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      value_idx = 0;
      value[0] = "0";
      value[1] = "0";
      value[2] = ".";
      value[3] = "0";
      value[4] = "0";
      value[5] = "\0";
    } 
    else {
      if (value_idx < 5) 
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  unsigned long current_millis = millis();

  if (current_millis - last_millis >= interval) 
  {
    left_wheel_measured_vel = 10 * left_encoder_counter * (60.0 / 385.0) * 0.10472;
    right_wheel_measured_vel = 10 * right_encoder_counter * (60.0 / 385.0) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();
    if (right_wheel_cmd_vel == 0) 
    {
      right_wheel_cmd = 0;
    }
    if (left_wheel_cmd_vel == 0) 
    {
      left_wheel_cmd = 0;
    }

    String encoder_reaad = "r" + right_encoder_sign + String(right_wheel_measured_vel) + ",l" + left_encoder_sign + String(left_wheel_measured_vel) + ",";
    Serial.println(encoder_reaad);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(left_enb, left_wheel_cmd);
    analogWrite(right_enb, right_wheel_cmd);
  }
}
