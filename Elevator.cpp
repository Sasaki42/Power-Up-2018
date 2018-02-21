/*
 * Elevator.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: srvro
 */

#include "Elevator.h"
#include "WPILib.h"

Elevator::Elevator(uint Elevator_Motor1_CH, uint Elevator_Motor2_CH) {
	pElvMotor1 = new Spark(Elevator_Motor1_CH);
	pElvMotor2 = new Spark(Elevator_Motor2_CH);
}
Elevator::~Elevator() {
	// TODO Auto-generated destructor stub
}
void Elevator::Up1(float speed){
	//pElvMotor1->Set(speed);
	if(speed>0){
	pElvMotor2->Set(Motor_Speed_Up);
	}
	else{
	pElvMotor2->Set(Motor_Speed_Down);
	}
	}
/*
void Elevator::Down1(float speed){
	pElvMotor1->Set(speed);
}
*/

void Elevator::Up2(float speed){
	//pElvMotor1->Set(speed);

	if(speed>0){
	pElvMotor2->Set(Motor_Speed_Up);
	}
	else{
	pElvMotor2->Set(Motor_Speed_Down);
	}
}
/*
void Elevator::Down2(float speed){
	pElvMotor2->Set(speed);
}*/
/*
void Elevator::Down(float speed){
	pElvMotor1->Set(speed);
	pElvMotor2->Set(speed);
}
*/
void Elevator::Stop(){
	pElvMotor1->Set(Motor_Speed_Stop);
}

void Elevator::Stop2(){
	pElvMotor2->Set(Motor_Speed_Stop);
}
