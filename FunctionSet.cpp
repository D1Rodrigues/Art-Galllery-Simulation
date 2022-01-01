/*
 * @Date: Dec. 15, 2021
*/

#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "Setup.h"
bool sensors_on = true;
bool line_event = false;
bool obstacle_event = false;

#include "ArduinoJson-v6.11.1.h" //ArduinoJson

/*Initializing pins*/
void BotFunctionSet::BotMovement_Init(void) //initialize bot for movement
{
  Serial.begin(9600);
  /* assign mins to motors */
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN, OUTPUT);
  pinMode(PIN_Motor_BIN, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH); //turn on bot and motors
}

/* initialize the IR sensor */
void BotFunctionSet::Sensors_Init(void)
{
  Wire.begin();
  Wire.beginTransmission(IRSensor_Address);
  Wire.write(110);
  Wire.endTransmission();
}

/* Move bot forward */
void BotFunctionSet::Move_Forward(uint8_t speed)
{
  digitalWrite(PIN_Motor_AIN, HIGH); //right motor turned on
  digitalWrite(PIN_Motor_BIN, LOW); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, speed); //right motor
  analogWrite(PIN_Motor_PWMB, speed); //left motor
}

/*move bot backward */
void BotFunctionSet::Move_Backward(uint8_t speed)
{
  digitalWrite(PIN_Motor_AIN, LOW); //right motor turned on
  digitalWrite(PIN_Motor_BIN, HIGH); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, speed); //right motor
  analogWrite(PIN_Motor_PWMB, speed); //left motor
}

/* stop bot */
void BotFunctionSet::Stop_Moving(void)
{
    digitalWrite(PIN_Motor_STBY, LOW); //turn off
    analogWrite(PIN_Motor_PWMA, 0);
    analogWrite(PIN_Motor_PWMB, 0);
}


void BotFunctionSet::Turn_Left(void) //turns bot to the left
{
  digitalWrite(PIN_Motor_AIN, HIGH); //makes the right motor move 
  digitalWrite(PIN_Motor_BIN, HIGH); // left motor is still
  analogWrite(PIN_Motor_PWMA, 150); //you can adjust the speed
  analogWrite(PIN_Motor_PWMB, 10);  
}

void BotFunctionSet::Turn_Right(void) //turns bot to the right
{
  digitalWrite(PIN_Motor_AIN, LOW); //makes the right motor move - left motor is still
  digitalWrite(PIN_Motor_BIN, LOW);
  analogWrite(PIN_Motor_PWMA, 10);
  analogWrite(PIN_Motor_PWMB, 150);
}

void Move_LeftForward(void) //move forward but turned to the left
{
  digitalWrite(PIN_Motor_AIN, HIGH); //right motor turned on
  digitalWrite(PIN_Motor_BIN, LOW); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, 170); //right motor
  analogWrite(PIN_Motor_PWMB, 130); //left motor
}

void Move_LeftBackward(void) //move backward but to the left
{
  digitalWrite(PIN_Motor_AIN, LOW); //right motor turned on
  digitalWrite(PIN_Motor_BIN, HIGH); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, 170); //right motor
  analogWrite(PIN_Motor_PWMB, 140); //left motor
  
}
void Move_RightForward(void) //move forward but to the right
{
  digitalWrite(PIN_Motor_AIN, HIGH); //right motor turned on
  digitalWrite(PIN_Motor_BIN, LOW); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, 120); //right motor
  analogWrite(PIN_Motor_PWMB, 170); //left motor
}

void Move_RightBackward(void) //move backward to the right
{
  digitalWrite(PIN_Motor_AIN, LOW); //right motor turned on
  digitalWrite(PIN_Motor_BIN, HIGH); //left motor turned on
  /* set the speed for the motors based on the input */
  analogWrite(PIN_Motor_PWMA, 125); //right motor
  analogWrite(PIN_Motor_PWMB, 170); //left motor
}

void BotFunctionSet::Stop_Gradually(void)
{
  for (int i = 0; i < 8; i++)
  {
    Move_Forward(normal_speed - 17);
  }
  Stop_Moving();
}

bool BotFunctionSet::Line_Checking(void)
{
  /*-------------------------------- line tracking and course correction ---------------------------- */
  uint8_t sensor_buffer[8];
  uint8_t buffer_index = 0;
  static uint8_t cout = 0;
  Wire.requestFrom(IRSensor_Address, 8); // request 6 bytes from slave device #2
  volatile uint16_t TrackingData_RightSensor;    
  volatile uint16_t TrackingData_LeftSensor;
  volatile uint16_t TrackingData_MiddleSensor;
  while (Wire.available())                    // slave may send less than requested
  {
    sensor_buffer[buffer_index++] = Wire.read(); // receive a byte as character
  }
  delay(10);
  if ((sensor_buffer[0] == 0XA0) && (sensor_buffer[7] == 0XB0))
  {
    TrackingData_RightSensor = (sensor_buffer[1] << 8) | (sensor_buffer[2]);
    TrackingData_LeftSensor = (sensor_buffer[3] << 8) | (sensor_buffer[4]);
    TrackingData_MiddleSensor = (sensor_buffer[5] << 8) | (sensor_buffer[6]);

    if(TrackingData_MiddleSensor > 200 && TrackingData_MiddleSensor < 860 && TrackingData_RightSensor > 200 && TrackingData_RightSensor < 860)
    {
      Move_Backward(255);
      delay(300);
      Turn_Left();
      delay(150); 
      return true;
    }

   if(TrackingData_MiddleSensor > 200 && TrackingData_MiddleSensor < 860 && TrackingData_LeftSensor > 200 && TrackingData_LeftSensor < 860)
   {
      Move_Backward(255);
      delay(300);
      Turn_Right();
      delay(150); 
      return true;
   }
            
    if (TrackingData_RightSensor > 200 && TrackingData_RightSensor < 860)
    {
      Turn_Left();
      delay(150); 
      return true;
    }

   else if(TrackingData_LeftSensor > 200 && TrackingData_LeftSensor < 860)
   {
      Turn_Right();
      delay(150); 
      return true;
   }

   else if(TrackingData_MiddleSensor > 200 && TrackingData_MiddleSensor < 860)
   {
      Move_Backward(255);
      delay(300);
      Turn_Right();
      delay(150);
      return true; 
   }
   else 
   {
    return false;
   }
    cout = 0;
  }
  else
  {
    /* in case of errors */
    cout += 1; //index is incremented each time there is an error?
    if (cout > 10) //if the numbers of errors is greater than 10...
    {
      Wire.beginTransmission(IRSensor_Address);
      Wire.write(110);
      Wire.endTransmission(); //stops the sensor
    }
    Stop_Moving();
    return true;
  }

  
}

bool BotFunctionSet::Obstacle_Avoidance(void)
{
  /* ------------------------------obstacle avoidance----------------------------- */
  unsigned dat[2] = {0};
  uint16_t ULTRASONIC_Get;
  volatile uint16_t ULTRASONIC_Data; 
  Wire.requestFrom(ULTRASONIC_Address, 1); 
  if (Wire.available() > 0)
  {
    dat[0] = Wire.read();
  }
  Wire.requestFrom(ULTRASONIC_Address, 1); 
  if (Wire.available() > 0)
  {
    dat[1] = Wire.read();
  }

  ULTRASONIC_Get = ((dat[0] << 8) | dat[1]);
  ULTRASONIC_Data = ULTRASONIC_Get / 10;
  Serial.println(ULTRASONIC_Data);
  delay(10);

  if (ULTRASONIC_Data < 15 && ULTRASONIC_Data > 0 && sensors_on)
  { 
      Stop_Gradually();
      delay(300);
      return true;
  }
  else 
  {
    return false;
  }
  
}

/*main function */
void BotFunctionSet::Rocker_Main(void) 
{
  
  /* -------------------------- section for driver control   -------------------- */
  String SerialPortData = ""; //variable to store data from the serial port

  /* receives and stores data from serial port */
  while ((Serial.available() > 0) && (false == SerialPortData.endsWith("}")))
  {
    SerialPortData += char(Serial.read());
    delay(6); /*Necessary delay to prevent data loss*/
  }
  //bool sensors_on = true;
  if ((SerialPortData.length() > 0) && (SerialPortData != "") && (true == SerialPortData.endsWith("}"))) //if serial port data is correct, proceed 
  {
    /*Import and create JsonDocument */
    StaticJsonDocument<200> json_doc; //Create a JsonDocument object
    DeserializationError error_check = deserializeJson(json_doc, SerialPortData); //Deserialize the JSON data
    if (!error_check) //check if there are no errors from the deserialization of the JSON data
    {
      int control_mode = json_doc["N"]; //gets "N" instructions
      char cmd_buffer[3];
      uint8_t temp = json_doc["H"]; //gets a "H:" instruction from app...
      sprintf(cmd_buffer, "%d", temp); //...and sends it back to tell app to execute next instruction (?)
      //analyze and move bot according to the commands from app via Bluetooth
      switch (control_mode) 
      {
        case 100:                                                                      
          Stop_Moving(); /*stop the bot*/
          break;

        case 101: 
          if (2 == json_doc["D1"]) //top button
          {
            sensors_on = false;
            delay(50);
            break;
          }
          else if (1 == json_doc["D1"]) //bottom button
          {
             sensors_on = true;
             delay(50);
             break;
          }
 
        case 102: /*controlling the joystick*/
          uint8_t Rocker_temp = json_doc["D1"];
          switch (Rocker_temp) //based on D1 command, move bot in appropiate direction
          {
            
            case 1:
              if (sensors_on)
              { 
                  line_event = Line_Checking();
                  obstacle_event = Obstacle_Avoidance();
              }

              if (!line_event && !obstacle_event)
              {
              Move_Forward(normal_speed);
              }

              if(!sensors_on)
              {
                Move_Forward(normal_speed);
              }
              break;
            case 2:
              Move_Backward(normal_speed);
              break;
            case 3:
              if (sensors_on)
              { 
                  line_event = Line_Checking();
                  obstacle_event = Obstacle_Avoidance();
              }
              if (!line_event && !obstacle_event)
              {
              Turn_Left();
              }
              if (!sensors_on)
              {
              Turn_Left();
              }
              break;
            case 4:
              if (sensors_on)
              { 
                  line_event = Line_Checking();
                  obstacle_event = Obstacle_Avoidance();
              }
              if (!line_event && !obstacle_event) {
              Turn_Right();
              }
              if (!sensors_on) {
              Turn_Right();
              }
              break;
            case 5:
              if(sensors_on) { obstacle_event = Obstacle_Avoidance();
                               line_event = Line_Checking(); };
              if (!sensors_on) { Move_LeftForward(); }
              if (!obstacle_event && !line_event) {
              Move_LeftForward();
              }
              if (!sensors_on) { Move_LeftForward(); }
              break;
            case 6:
              Move_LeftBackward();
              break;
            case 7:
              if(sensors_on) { obstacle_event = Obstacle_Avoidance();
                               line_event = Line_Checking(); };
              if (!sensors_on) { Move_RightForward(); }
              if (!obstacle_event && !line_event) {
              Move_RightForward();
              }
              if (!sensors_on) { Move_RightForward(); }
              break;
            case 8:
              Move_RightBackward();
              break;
            case 9:
              Stop_Moving();
              break;
            default:
              Stop_Moving();
          break;
          }
            break;
          default:
            break;
      }
    }
  }
}
