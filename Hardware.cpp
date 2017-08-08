/*

 * hardware.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: Developer
 */
#include "hardware.h"
#include "WPILib.h"
#include "CANTalon.h"


Hardware::Hardware()
{

   // enum  {PORT0,PORT1,PORT2,PORT3,PORT4,PORT5,PORT6,PORT7,PORT8,PORT9};
    const int PWM_PORT0 = 0;
    const int PWM_PORT1 = 1;
	const int PWM_PORT2 = 2;
	const int PWM_PORT3 = 3;
	const int PWM_PORT4 = 4;
	const int PWM_PORT5 = 5;
	const int PWM_PORT6 = 6;
	const int PWM_PORT7 = 7;
	const int PWM_PORT8 = 8;
	const int PWM_PORT9 = 9;


//    enum {PORT_0,PORT_1,PORT_2,PORT_3,PORT_4,PORT_ 5,PORT_6,PORT_7,PORT_8,PORT_9};  // enum DIO
    const int DIO_PORT0 = 0;
    const int DIO_PORT1 = 1;
    const int DIO_PORT2 = 2;
    const int DIO_PORT3 = 3;
    const int DIO_PORT4 = 4;
    const int DIO_PORT5 = 5;
    const int DIO_PORT6 = 6;
    const int DIO_PORT7 = 7;
    const int DIO_PORT8 = 8;
    const int DIO_PORT9 = 9;


	flywheel = new CANTalon(4);//CANloop_FLYWHEEL
	conveyor = new Spark(PWM_PORT0);
	kicker = new Spark(PWM_PORT1);
	lFlap = new Servo(PWM_PORT7);
	rFlap = new Servo(PWM_PORT8);
	climber = new Spark(PWM_PORT4);
	intake = new Spark(PWM_PORT5);
	hopper = new Servo(PWM_PORT6);
	hopperBouncer1 = new Spark(PWM_PORT2);
	hopperBouncer2 = new Spark(PWM_PORT3);



	navx = new AHRS(SerialPort::Port::kMXP);
	wheelEncoder = new Encoder(DIO_PORT6,DIO_PORT7,false,Encoder::EncodingType::k4X);
	shooterEncoder = new Encoder (DIO_PORT8,DIO_PORT9,false,Encoder::EncodingType::k4X);
	shooterEncoder->SetDistancePerPulse(.0654);


	kickerLimit = new DigitalInput(DIO_PORT4);
	gearButton = new DigitalInput(DIO_PORT5); //  DIRECT STOP FOR BOT1 THEREFORE NO NEED FOR DIGITAL LIMIT

	kickTimer = new Timer();

}


