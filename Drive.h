#ifndef DRIVE_H
#define DRIVE_H
#include "CANTalon.h"
#include "WPILib.h"

class Drive
{
private:
	RobotDrive* driver;
public:
	Drive();
	CANTalon *Rleft;
	CANTalon *Fleft;
	CANTalon *Fright;
	CANTalon *Rright;

	void AutoArcade(float x, float y);
	void AutoDrive(float x, float y);
	void setDriveControl(XboxController *xbox);
	void setDriveControl2(XboxController *xbox);
};
#endif
