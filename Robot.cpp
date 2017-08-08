#include <Drive.h>
#include "WPILib.h"
#include "hardware.h"
#include <Math.h>


class Robot:
public IterativeRobot
{
private:
	Drive* driver;               //defines variables
	Hardware* hardware;
	Timer* t;
	Timer* gearReset;
	XboxController* xboxDrive;
	Joystick * joystickAux;
	Joystick* joystickTest = new Joystick(2);
	int Treset;
	double circumference;
	double lFlapOpen,rFlapOpen,lFlapClosed,rFlapClosed;
	bool gearMechOpen;
	double leftServo;
	double rightServo;
	double hopperServo=.5;
	double shooterspeed;
	enum {A_Button,B_BBUTTON,X_BUTTON,Y_BUTTON,LEFT_BUMPER,RIGHT_BUMPER,XBOX_TAB,XBOX_START,LEFT_JOYBUTTON,RIGHT_JOYBUTTON};
	enum {LEFT_X,LEFT_Y,LEFT_TRIGGER,RIGHT_TRIGGER,RIGHT_X,RIGHT_Y};
	enum {JOYSTICK_TRIGGER=1,SIDE_BUTTON,THUMB_BOTlEFT,THUMB_BOTRIGHT,THUMB_TOPLEFT,THUMB_TOPRIGHT};
	enum {X_AXIS,Y_AXIS,TWIST_AXIS,SLIDER};
	double fineAdjust;
	bool flyWheelOn;
	double shooter;
	bool gear;
	double target = .75,threshhold = .05,minimumPower =.7 ,MaxPower = 1;
	double turningKP;
	double turningKD;


public:
	Robot()
	{
		turningKP = SmartDashboard::GetNumber("Turning KP",.1);
		turningKD = SmartDashboard::GetNumber("Turning KD",.1);
		gearReset = new Timer();
		t = new Timer();
		hardware= new Hardware();
		driver= new Drive();             //initiates robot components
		Treset = 5;
		circumference = 0;
		shooter = 0;
		lFlapOpen = SmartDashboard::GetNumber("LeftFlap Open", .22);
		rFlapOpen = SmartDashboard::GetNumber("RightFlap Open", .87);
		lFlapClosed = SmartDashboard::GetNumber("LeftFlap Closed", .36);
		rFlapClosed = SmartDashboard::GetNumber("RightFlap Closed", .75);
		leftServo= lFlapClosed;
		rightServo = rFlapClosed;
		xboxDrive = new XboxController(0);
		joystickAux = new Joystick(1);
		gearMechOpen = false;
		fineAdjust = 0;
		flyWheelOn = false;
		gear = true;
		shooterspeed = hardware->shooterEncoder->GetRate();
	}

	void AutoDrive(double l,double r)
	{
		driver->Fleft->Set(l);
		driver->Rleft->Set(l);
		driver->Fright->Set(-r);
		driver->Rright->Set(-r);
	}
	void turnRight (int turnDegrees)
	{
		hardware->navx->Reset();
		double startingAngle = hardware->navx->GetAngle(),power = 1;

		while (startingAngle+turnDegrees-3 > hardware->navx->GetAngle() && power >= 0.35)
		{
			double currAngle = hardware->navx->GetAngle();
			power = 1/(0.03115*currAngle+1);
			AutoDrive(power,-power);

		}
		while(startingAngle+turnDegrees-3 > hardware->navx->GetAngle())
		{
			AutoDrive(power,-power);
		}
		AutoDrive(0,0);
	}

	void turnLeft (int turnDegrees)
	{
		hardware->navx->Reset();
		double startingAngle = hardware->navx->GetAngle(),power = 1;

		while(startingAngle+turnDegrees-3 > abs(hardware->navx->GetAngle()) && power >= 0.35)
		{
			double currAngle = abs(hardware->navx->GetAngle());
			power = 1/(0.03115*currAngle+1);
			AutoDrive(-power,power);

		}
		while(startingAngle+turnDegrees-3 > abs(hardware->navx->GetAngle()))
		{
			AutoDrive(-power,power);
		}
		AutoDrive(0,0);
	}
	void PDturnRight (double turnDegrees)
	{
		hardware->navx->Reset();
		double startingAngle = hardware->navx->GetAngle(),power,threshold = 0.05;
		double error,oldError = 0;
		double derivative,PID,PID_sat = 5;
		double deltaT = 10;

		while(abs(turnDegrees-hardware->navx->GetAngle())> threshold)
		{
			//t->Reset();
			//t->Start();

			error = turnDegrees-hardware->navx->GetAngle();

			derivative = (error-oldError)/deltaT;

			PID = -turningKP*error-turningKD*derivative;

			if(abs(PID)>PID_sat)
			{
				power = 0.8;
			}

			else
			{
				power = 0.8*(abs(PID)/PID_sat);
			}

			AutoDrive(power,-power);

			error = oldError;
			Wait(deltaT/1000);
			//t->Stop();
			//deltaT = t->Get();
		}
	AutoDrive(0,0);
	}
	void PDturnLeft (double turnDegrees)
	{
		hardware->navx->Reset();
		double startingAngle = hardware->navx->GetAngle(),power,threshold = 0.05;
		double error,oldError = 0;
		double derivative,PID,PID_sat = 5;
		double deltaT = 10;



		while(abs(turnDegrees-hardware->navx->GetAngle())> threshold)
		{
		//	t->Reset();
		//	t->Start();


			error = turnDegrees-hardware->navx->GetAngle();

			derivative = (error-oldError)/deltaT;

			PID = -turningKP*error-turningKD*derivative;

				if(abs(PID)>PID_sat)
				{
					power = 0.8;
				}

				else
				{
					power = 0.8*(abs(PID)/PID_sat);
				}

			AutoDrive(-power,power);

			error = oldError;
			Wait(deltaT/1000);
		//	t->Stop();
		//	deltaT = t->Get();

		}
		AutoDrive(0,0);
	}

	void EncoderStraightDrive(double speed, double distance, double deltaTime)
	{
		double kP = -SmartDashboard::GetNumber("kP", .1);
		double kD = -SmartDashboard::GetNumber("kD", .1);

		hardware->navx->Reset();
		hardware->wheelEncoder->Reset();


		double error = 0;
		double errornew;
		double change;
		double derivative;



		while(abs(hardware->wheelEncoder->GetDistance()) <= distance-5&&hardware->gearButton->Get()==true)
		{
			errornew = hardware->navx->GetAngle()-6;

			change = errornew-error;

			derivative = change/deltaTime;

			driver->AutoArcade(speed, -(kP*errornew + kD*derivative));

			error = errornew;
		}
		driver->AutoArcade(0,0);
	}


	void TimeStraightDrive(double speed, double seconds, int TReset)
	{
		double kP = -SmartDashboard::GetNumber("kP", .1);
		double kD = -SmartDashboard::GetNumber("kD", .1);

		int deltaTime = TReset; //milliseconds
		t->Reset();
		t->Start();
		hardware->navx->Reset();
		hardware->wheelEncoder->Reset();

		double error = 0;
		double errornew;
		double change;
		double derivitive;

		while(t->Get() <= seconds&&hardware->gearButton->Get()==true)
		{
			errornew = hardware->navx->GetAngle()-SmartDashboard::GetNumber("Autonomous Navx offset",1);

			change = errornew-error;

			derivitive = change/(double)deltaTime;

			driver->AutoArcade(speed, -(kP*errornew + kD*derivitive));

			error = errornew;
		}
		driver->AutoArcade(0,0);
		t->Stop();
	}


	void toggleFlywheel()
	{
		if (flyWheelOn==false)
			flyWheelOn = true;
		else
			flyWheelOn = false;
		Wait(.15); //Double check if this interferes AL
	}
	void BANGBANG(double target)
	{

		 if (shooterspeed <= target-threshhold)
			hardware->flywheel->Set(minimumPower);

		 else if (shooterspeed >= target+threshhold)
			hardware->flywheel->Set(MaxPower);

	}
	void toggleGearMech()
	{
		if(gearMechOpen)
		{
			hardware->lFlap->Set(lFlapClosed);
			hardware->rFlap->Set(rFlapClosed);
			gearMechOpen=false;
		}
		else
		{
			hardware->lFlap->Set(lFlapOpen);
			hardware->rFlap->Set(rFlapOpen);
			gearMechOpen=true;
		}
	}
	void gearSequence()
	{
		driver->setDriveControl(xboxDrive);
		if(gearMechOpen==false)
			toggleGearMech();
		Wait(.4);
		kick();
		toggleGearMech();
	}
	void kick ()
	{

		hardware->kickTimer->Reset();
		hardware->kickTimer->Start();
		while (hardware->kickTimer->Get() < .5&&(hardware->kickerLimit->Get()==false))
		{
			hardware->kicker->Set(1);
			driver->setDriveControl(xboxDrive);
		}
		hardware->kickTimer->Reset();
		hardware->kickTimer->Start();
		while (hardware->kickTimer->Get() < 1)
		{
			hardware->kicker->Set(-.4);
			driver->setDriveControl(xboxDrive);
		}
		hardware->kicker->Set(0);
	}
	void RobotPeriodic()
	{
		SmartDashboard::PutNumber("Shooter Encoder Reading",hardware->shooterEncoder->GetDistance());
		SmartDashboard::PutNumber("Wheel Encoder Reading",hardware->wheelEncoder->GetDistance());
		SmartDashboard::PutNumber("Shooter Speed", shooter);
		SmartDashboard::PutNumber("Navx GetAngle",hardware->navx->GetAngle());
		SmartDashboard::PutNumber("LeftFlap Value", leftServo);
		SmartDashboard::PutNumber("RightFlap Value", rightServo);
		lFlapOpen = SmartDashboard::GetNumber("LeftFlap Open", .22);
		rFlapOpen = SmartDashboard::GetNumber("RightFlap Open", .87);
		lFlapClosed = SmartDashboard::GetNumber("LeftFlap Closed", .36);
		rFlapClosed = SmartDashboard::GetNumber("RightFlap Closed", .75);
	}
	void RobotInit()
	{
		//autonomous variables
		SmartDashboard::PutNumber("Shooter Target", target);
		SmartDashboard::PutNumber("kP", .05);
		SmartDashboard::PutNumber("kD", .1);
		SmartDashboard::PutNumber("Wheel Diameter (in inches)", 5.875);
		SmartDashboard::PutNumber("starting position",1);
		SmartDashboard::PutString("alliance color", "blue");
		SmartDashboard::PutNumber("Autonomous v1 v2 v3",1);
		SmartDashboard::PutNumber("Shooter kP1", 0.1);
		SmartDashboard::PutNumber("Shooter kI1", 0.1);
		SmartDashboard::PutNumber("LeftFlap Open", .22);
		SmartDashboard::PutNumber("RightFlap Open", .87);
		SmartDashboard::PutNumber("LeftFlap Closed", .36);
		SmartDashboard::PutNumber("RightFlap Closed", .75);
		SmartDashboard::PutNumber("Turning KP",.1);
		SmartDashboard::PutNumber("Turning KD",.1);
		hardware->lFlap->Set(lFlapClosed);
		hardware->rFlap->Set(rFlapClosed);
		circumference = M_PI*SmartDashboard::GetNumber("Wheel Diameter (in inches)", 5.875);
		hardware->wheelEncoder->SetDistancePerPulse(circumference*3.14/(1600.0*.9));     //15/22 is gear ratio
	}

	void AutonomousInit()
	{
		double autoSpeed = .6;
		int startPos = SmartDashboard::GetNumber("starting position", 1.0);
		if (SmartDashboard::GetNumber("Autonomous v1 v2 v3",1)==1)        //v1 is EncoderStraightDrive, v2 is Timers, v3 is nothing
		{
			if (SmartDashboard::GetString("alliance color","blue")=="blue")
			{
				switch(startPos)
				{
				case 1:// left blue
					EncoderStraightDrive(autoSpeed,105.4,Treset);
					turnRight(60);
					EncoderStraightDrive(autoSpeed,18.8,Treset);
					gearSequence();
					AutoDrive(-.8,-.8);
					Wait(.9);
					AutoDrive(0,0);
					turnRight(30);
					/*EncoderStraightDrive(-autoSpeed,69.6,Treset);
					turnRight(90);
					EncoderStraightDrive(autoSpeed,25.2,Treset);
					turnRight(77.6);
					EncoderStraightDrive(autoSpeed,47.2,Treset);
					 //flyWheelOn*/
					break;
				case 2:// middle blue
					EncoderStraightDrive(autoSpeed,83,Treset);
					gearSequence();
					AutoDrive(-.7,-.7);
					Wait(.7);
					AutoDrive(0,0);
					/*EncoderStraightDrive(-autoSpeed,31.52,Treset);
					turnLeft(90);
					EncoderStraightDrive(autoSpeed,95.2,Treset);
					turnLeft(44.2);
					EncoderStraightDrive(autoSpeed,47.2,Treset);
					 //flyWheelOn*/
					 break;
				case 3: // right blue
					EncoderStraightDrive(autoSpeed,69.6,Treset);
					turnLeft(60);
					EncoderStraightDrive(autoSpeed,58.4,Treset);
					gearSequence();
					AutoDrive(-.7,-.7);
					Wait(.7);
					AutoDrive(0,0);
					break;

				default:
					 break;

				}


			}

			else if (SmartDashboard::GetString("alliance color","blue")=="red")
			{
				switch(startPos)
				{


				case 1: // left red
					EncoderStraightDrive(autoSpeed,65.6,Treset);
					turnRight(60);
					EncoderStraightDrive(autoSpeed,52.8,Treset);
					gearSequence();
					AutoDrive(-.7,-.7);
					Wait(.3);
					AutoDrive(0,0);

					break;

				case 2:// middle red
					EncoderStraightDrive(autoSpeed,83,Treset);
					gearSequence();
					AutoDrive(-.7,-.7);
					Wait(.7);
					AutoDrive(0,0);
					/*EncoderStraightDrive(-autoSpeed,31.52,Treset);
					turnRight(90);
					EncoderStraightDrive(autoSpeed,58.2,Treset);
					turnRight(44.7);
					EncoderStraightDrive(autoSpeed,47.2,Treset);
					//flyWheelOn*/
					break;
				case 3:// right red
					EncoderStraightDrive(autoSpeed,72.4,Treset);
					turnLeft(60);
					EncoderStraightDrive(autoSpeed,52.8,Treset);
					gearSequence();
					AutoDrive(-.7,-.7);
					Wait(.7);
					AutoDrive(0,0);
					/* EncoderStraightDrive(-autoSpeed,69.6,Treset);
					 turnLeft(90);
					 EncoderStraightDrive(autoSpeed,25.2,Treset);
					 turnLeft(77.6);
					 EncoderStraightDrive(autoSpeed,47.2 ,Treset);
					 //flyWheelOn*/
					break;

				default:
					 break;

				}
			}

		}

		else if(SmartDashboard::GetNumber("Autonomous v1 v2 v3",1)==2.0)
		{
			turnRight(90);
			turnLeft(90);
		}
		else
		{

		}
	}




	void TeleopPeriodic()
	{
		if(hardware->gearButton->Get()==0&&gear)
		{
			t->Start();
			if(t->Get()<2)
				gearSequence();
			else
				gear=false;
			t->Stop();
		}
		if(joystickAux->GetRawButton(THUMB_BOTRIGHT)==1)
			hardware->hopper->Set(1);
		else
			hardware->hopper->Set(0);
		shooter = .7+ .3*(-.5*(joystickAux->GetRawAxis(SLIDER)-1))+fineAdjust;



		//Drive Controller
		driver->setDriveControl(xboxDrive);
		hardware->climber->Set(xboxDrive->GetRawAxis(LEFT_TRIGGER));
		if(xboxDrive->GetRawAxis(RIGHT_TRIGGER)||flyWheelOn)
		{
			hardware->hopperBouncer1->Set(1);
			hardware->hopperBouncer2->Set(1);
			hardware->intake->Set(-1);
		}
		else
		{
			hardware->hopperBouncer1->Set(0);
			hardware->hopperBouncer2->Set(0);
			hardware->intake->Set(0);
		}



		//Auxiliary Joystick
		if(joystickAux->GetRawButton(JOYSTICK_TRIGGER)==1)   //button 1 activates conveyor
			hardware->conveyor->Set(1);
		else
			hardware->conveyor->Set(0);

		if (joystickAux->GetRawButton(SIDE_BUTTON)==1)   //button 2 toggles intake, jostlers, and flywheel
			toggleFlywheel();
		if (joystickAux->GetRawButton(THUMB_TOPRIGHT)== 1)
			toggleGearMech();
		if (joystickAux->GetRawButton(THUMB_TOPLEFT)==1&&gearMechOpen)
			kick();
		if(joystickAux->GetRawButton(8)==1)
		{
			t->Reset();
			gear=true;
		}
		if(gear==false)
		{
			if(gearReset->Get()==0)
				gearReset->Start();
			if(gearReset->Get()>5)
			{
				t->Reset();
				gearReset->Reset();
				gear=true;
			}
		}



		if(flyWheelOn)
		{
			BANGBANG(shooter);
		}
		else
			hardware->flywheel->Set(0);


		if(joystickAux->GetRawButton(11)==1)
		{
			leftServo+=.005;
			hardware->lFlap->Set(leftServo);
		}
		if(joystickAux->GetRawButton(12)==1)
		{
			leftServo-=.005;
			hardware->lFlap->Set(leftServo);
		}
		if(joystickAux->GetRawButton(9)==1)
		{
			rightServo+=.005;
			hardware->rFlap->Set(rightServo);
		}
		if(joystickAux->GetRawButton(10)==1)
		{
			rightServo-=.005;
			hardware->rFlap->Set(rightServo);
		}


		if(joystickTest->GetRawButton(7)==1)
			PDturnRight(90);
		if(joystickTest->GetRawButton(9)==1)
			hardware->wheelEncoder->Reset();
		if(joystickTest->GetRawButton(2)==1)
			hardware->navx->Reset();
		if(joystickTest->GetRawButton(8)==1)
			PDturnLeft(90);

	}

};

START_ROBOT_CLASS(Robot);
