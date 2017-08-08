#include <Drive.h>
#include "WPILib.h"

Drive::Drive()
{
	enum {CAN0,CAN1,CAN2,CAN3,CAN4};
	Rleft = new CANTalon(CAN0);
	Fleft = new CANTalon(CAN1);
	Rright = new CANTalon(CAN2);
	Fright = new CANTalon(CAN3);
	driver = new RobotDrive(Rleft,Fleft,Fright,Rright);
	driver->SetExpiration(50);
	Rleft->SetSafetyEnabled(false);
	Fleft->SetSafetyEnabled(false);
	Rright->SetSafetyEnabled(false);
	Fright->SetSafetyEnabled(false);
	Rleft->SetExpiration(30);
	Fleft->SetExpiration(30);
	Rright->SetExpiration(30);
	Fright->SetExpiration(30);
}
void Drive::AutoArcade(float x, float y)
{
	driver->ArcadeDrive(x, y);
}
void Drive::AutoDrive(float L,float R)
{
	Rleft->Set(L);
	Fleft->Set(L);
	Rright->Set(-R);
	Fright->Set(-R);
}
 void Drive::setDriveControl(XboxController* xbox)
{
	//"GTA5/Halo" configuration
	//driver->ArcadeDrive(-.8*(xbox->GetRawAxis(3)-xbox->GetRawAxis(2)),xbox->GetRawAxis(0));


	//"Ben's configuration"
	 driver->ArcadeDrive(-xbox->GetRawAxis(1),-xbox->GetRawAxis(4));
}
