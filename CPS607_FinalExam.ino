/*
 * @Description: CPS607-FinalExam, 
 * @Author: Dylan Rodrigues
 * @Date: Dec. 15, 2021
*/


#include "Setup.h" //import all necessary functions

BotFunctionSet BotMovement;
void setup() {
  // initialize the OwlBot for movement:
  BotMovement.BotMovement_Init();
  BotMovement.Sensors_Init();
}

void loop() {
  // call the main function to run repeatedly:
  BotMovement.Rocker_Main();

}
