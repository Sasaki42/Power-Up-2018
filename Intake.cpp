/*
 * Intake.cpp
 *
 *  Created on: Jan 24, 2018
 *      Author: srvro
 */

#include "Intake.h"
#include "WPILib.h"

Intake::Intake(uint IntakeMotor1, uint IntakeMotor2)
{
	// TODO Auto-generated constructor stub
	MotorIntake1		= new Spark(IntakeMotor1);
	MotorIntake2		= new Spark(IntakeMotor2);
}

Intake::~Intake() {
	// TODO Auto-generated destructor stub
}

void Intake::In(){
		MotorIntake1->Set(Motor_Speed_Intake);
		MotorIntake2->Set(Motor_Speed_Intake);
}



void Intake::Out(){
	MotorIntake1->Set(Motor_Speed_Outtake);
	MotorIntake2->Set(Motor_Speed_Outtake);
}

void Intake::Stop(){
	MotorIntake1->Set(Motor_Speed_Stop);
	MotorIntake2->Set(Motor_Speed_Stop);
}
