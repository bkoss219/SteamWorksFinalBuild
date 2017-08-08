/*
 * hardware.h
 *
 *  Created on: Feb 1, 2016
 *      Author: Developer
 */
#ifndef HARDWARE_H
#define HARDWARE_H
#include "WPILib.h"
#include "CANTalon.h"
#include "AHRS.h"


class Hardware
{
public:
	Hardware();
	CANTalon*flywheel;
	Spark*conveyor;
	Spark*kicker;
	Servo*lFlap;
	Servo*rFlap;
	Spark*climber;
	Spark*intake;
	Servo*hopper;
	Spark*hopperBouncer1;
	Spark*hopperBouncer2;

	AHRS*navx;
	Encoder*wheelEncoder;
	Encoder*shooterEncoder;

	DigitalInput*kickerLimit;
	DigitalInput*gearButton;

	Timer*kickTimer;

 };
#endif


