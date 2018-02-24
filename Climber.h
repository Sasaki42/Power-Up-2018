/*
 * Climber.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Tyler
 */

#ifndef CLIMBER_H
#define CLIMBER_H

#include "WPILib.h"


class Climber
{

public:

	Climber(uint climbMotor1Ch, uint climbMotor2Ch);
	~Climber();

	void   Climb();
	void   Lower();
	float  GetMotor1Speed() const;
	float  GetMotor2Speed() const;
	void   StopClimber();

private:
	const float   MOTOR_SPEED_CLIMB		 =  1.00;
	const float   MOTOR_SPEED_LOWER		 = -1.00;
	const float   ALL_STOP                   =  0.00;

    	Spark        *pClimberMotor1;
	Spark        *pClimberMotor2;
};

#endif /* CLIMBER_H_ */
