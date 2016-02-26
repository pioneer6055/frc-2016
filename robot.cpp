#include "WPILib.h"

/*
 *  program for Pioneer (Team 6055) robot 2016
 *  version 1.01  2/2/2016  - CRM
 *      - initial deploy to robot
 *  version 1.02  2/11/2016  - CRM
 *      - added filtering to drivetrain inputs
 *      - added speed limiting when in a spin
 *      - corrected solenoid inits to include pcm CAN id number
 *      - removed servo and added another solenoid for trigger
 *      - remove cout and put all diagnostics to smartdashboard
 *  version 1.03  2/15/2016  - CRM
 *  	- add button 3 for shooter wheels (intake) with arm in down position
 *  	- make shooter wheels run automatically when trigger pressed
 *  	- make lag filter for drivetrain adjustable via throttle on joystick
 *  	- shooter wheels auto did not work - put on button 4
 *  version 1.04  2/16/2016  - CRM
 *  	-  changed timer values to large integers
 *  	-  again make shooter wheels run auto with delay on trigger - hopefully
 *  	-  remove smartdashboard and put all diagnostics via cout to netconsole
 *  	-  correct problem of not updating lastError in catch blocks
 *  	-  only check lag filter setting one time on entering OperatorControl
 *  	- update: removed timers because it was causing drive timeouts
 *  			  went back to seperate button for shooter wheels
 *  version 1.05  2/18/2016  - CRM
 *  	-  added check for intake and shooter buttons to not be pressed together
 *  	-  change motor safety expiration value to 250 msecs
 *  	-  make 2 functions for shooter - one for auto and one for buttons to test
 *  	-  the timeout problem was caused by not calling driveSpinner in each loop execution
 *  	-  fixed timeout problem by using variables to change speed and direction
 *  	-  moved lag filter setting back to loop
 *  version 1.06  2/19/2016  - CRM
 *  	-  added limit checking for joystick inputs
 *  	-  mechanical redesign - lost the arm
 *  	-  added button 4 for rotating spinners backward to shoot into low goal
 *  	-  tested robot speed at 17 feet in 1.43 seconds = 11.9 ft/sec
 *  version 1.07  2/23/2016  - CRM
 *  	-  turn on autonomous mode - travel 8ft @ 50% power = 1340 msecs
 *		-  remove compressor and solenoid code
 *		-  delete shooter_auto function
 *		-  widen limits for anti-spin
 *		-  add code for camera
 */
class Robot: public SampleRobot
{
	VictorSP motorLF, motorLR, motorRF, motorRR;
	VictorSP spinnerLeft, spinnerRight;
	RobotDrive driveTrain;
	RobotDrive driveSpinner;
	Joystick xbox, stick;
	PowerDistributionPanel pdp;
	float kUpdatePeriod = 0.005f; // loop delay of 5 milliseconds
	float sliceCount = 1.0f;
	float kMaxStickValue = 0.7f;
	float kMinStickValue = -0.7f;
	float throttle = 0.0f;
	float lagVal = 0.01f;  		  // lag filter value (.01 = 100 * .005 = .5 sec)
	float lagAdjust = 0.0f;
	float leftRaw = 0.0f;         // left side joystick input
	float leftVal = 0.0f;		  // left side drivetrain output
	float rightRaw = 0.0f;        // right side joystick input
	float rightVal = 0.0f;		  // right side drivetrain output
	float spinLeftVal = 0.0f;     // left side driveSpinner output
	float spinRightVal = 0.0f;	  // right side driveSpinner output
	float intakeSpeed = 0.25f;    // speed of motors for intake of ball
	float shootSpeed = 1.0f;      // speed of motors for shooting ball
	uint64_t autoTime = 0;
	uint64_t testTime = 0;
	std::string lastError = "";
	std::string nowError = "";

public:
	Robot() :
		motorLF(0),  		 //Left Front motor on PWM channel 0
		motorLR(1),  		 //Left Rear motor on PWM channel 1
		motorRF(2),  		 //Right Front motor on PWM channel 2
		motorRR(3),  		 //Right Rear motor on PWM channel 3
		spinnerLeft(4), 	 //left spinner motor on PWM channel 4
		spinnerRight(5), 	 //left spinner motor on PWM channel 5
		driveTrain(motorLF,motorLR,motorRF,motorRR),
		driveSpinner(spinnerLeft,spinnerRight),
		xbox(0),     		 //xbox on USB port 0
		stick(1),    		 //joystick on USB port 1
		pdp()				 //power distribution controller - CAN ID = 0
	{
	}

	void RobotInit()
	{
		std::cout << "RobotInit" << std::endl;
		//this sets the safety timeout for the driveTrain motors
		//loops in the functions below must not exceed this timeout value
		driveTrain.SetExpiration(0.25);
		driveSpinner.SetExpiration(0.25);
		//calculate program loops per second
		sliceCount = 1.0f/kUpdatePeriod;
		motorLF.SetInverted(false);
		motorLR.SetInverted(false);
		motorRF.SetInverted(false);
		motorRR.SetInverted(false);
		spinnerLeft.SetInverted(false);
		spinnerRight.SetInverted(false);
		std::cout << "Code Version:  1.07" << std::endl;
		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}

	void Autonomous()
	{
		if (IsAutonomous() && IsEnabled())
		{
			std::cout << "AutoMode START" << std::endl;
			driveTrain.SetSafetyEnabled(true);  //turn on safety watchdog for drive
			driveSpinner.SetSafetyEnabled(true);  //turn on safety watchdog for drive
			autoTime = GetFPGATime();
		}
		while (IsAutonomous() && IsEnabled())
		{
			testTime = GetFPGATime();
			testTime = (testTime - autoTime) / 1000;  //get milliseconds
			if (testTime < 3250)  //want to travel 8 feet
				driveTrain.TankDrive(-0.65,-0.65);   //drive forward
			else
				driveTrain.TankDrive(0.0,0.0); //stop
		}
		driveTrain.StopMotor();
		driveSpinner.StopMotor();
		std::cout << "AutoMode STOP" << std::endl;
	}

	void RunShooter()
	{
		try
		{
			spinLeftVal = 0;
			spinRightVal = 0;
			//run spinner motors when button pressed and not firing
			if (stick.GetRawButton(10) == true && stick.GetRawButton(3) == false && stick.GetRawButton(4) == false)
			{
				spinLeftVal = -1 * intakeSpeed;
				spinRightVal = intakeSpeed;
			}

			//run spinner motors to shoot ball forward - high goal
			if (stick.GetRawButton(3) == true && stick.GetRawButton(10) == false && stick.GetRawButton(4) == false)
			{
				spinLeftVal = -1 * shootSpeed;
				spinRightVal = shootSpeed;
			}

			//run spinner motors to shoot ball backward - low goal
			if (stick.GetRawButton(4) == true && stick.GetRawButton(10) == false && stick.GetRawButton(3) == false)
			{
				spinLeftVal = shootSpeed;
				spinRightVal = -1 * shootSpeed;
			}

			driveSpinner.TankDrive(spinLeftVal,spinRightVal,false);
			SmartDashboard::PutBoolean("DB/Button 0", spinLeftVal != 0);
		}
		catch (std::exception& ex )
		{
			nowError = ex.what();
			if (lastError != nowError)
			{
				std::cout << "[SHOOTER ERROR] " << nowError << std::endl;
				lastError = nowError;
			}
		}
	}

	void RunDriveTrain()
	{
		try
		{
			//grab lag filter setting from joystick throttle axis
			try
			{   //scale adjustment to 0-100% of axis (-1 to 1)
				float tmpfloat = stick.GetRawAxis(2);
				if (tmpfloat != throttle)
				{
					throttle = tmpfloat;
					std::cout << "throttle = " << throttle << std::endl;
					lagAdjust = (throttle + 1.0f) / 2.0f;
					std::cout << "lagAdjust = " << lagAdjust << std::endl;
					lagVal = 1.0f / (lagAdjust * sliceCount);
					std::cout << "lagVal = " << lagVal << std::endl;
					SmartDashboard::PutNumber("DB/Slider 1", lagVal);
				}
			}
			catch(std::exception& e )
			{
				lagVal = 0.01f;
			}
			//get raw joystick values from xbox
			leftRaw = xbox.GetRawAxis(1);
			rightRaw = xbox.GetRawAxis(5);
			//add lag filter to joystick values to limit acceleration
			leftVal = lagVal * leftRaw + ((1-lagVal) * leftVal);
			rightVal = lagVal * rightRaw + ((1-lagVal) * rightVal);
			//limit max joystick offset to limit speed in a spin
			if (leftVal > kMaxStickValue && rightVal < kMinStickValue)
			{
				leftVal = kMaxStickValue;
				rightVal = kMinStickValue;
			}
			if (rightVal > kMaxStickValue && leftVal < kMinStickValue)
			{
				rightVal = kMaxStickValue;
				leftVal = kMinStickValue;
			}
			if (leftVal > 0.99) leftVal = 0.99;
			if (leftVal <-0.99) leftVal = -0.99;
			if (rightVal > 0.99) rightVal = 0.99;
			if (rightVal < -0.99) rightVal = -0.99;
			driveTrain.TankDrive(leftVal,rightVal);
		}
		catch (std::exception& ex )
		{
			nowError = ex.what();
			if (lastError != nowError)
			{
				std::cout << "[DRIVETRAIN ERROR] " << nowError << std::endl;
				lastError = nowError;
			}
		}
	}

	void OperatorControl()
	{
		if (IsOperatorControl() && IsEnabled())
		{
			std::cout << "OpMode START" << std::endl;
			driveTrain.SetSafetyEnabled(true);  //turn on safety watchdog for drive
			driveSpinner.SetSafetyEnabled(true);  //turn on safety watchdog for drive
		}
		while (IsOperatorControl() && IsEnabled())
		{
			try
			{
				RunShooter();
				RunDriveTrain();
				SmartDashboard::PutNumber("DB/Slider 0", pdp.GetTotalCurrent());
			}
			catch (std::exception& ex )
			{
				nowError = ex.what();
				if (lastError != nowError)
				{
					std::cout << "[OPMODE ERROR] " << nowError << std::endl;
					lastError = nowError;
				}
			}
			Wait(kUpdatePeriod); // Wait delay time before looping
		}
		driveTrain.StopMotor();
		driveSpinner.StopMotor();
		std::cout << "OpMode STOP" << std::endl;
	}

	void Test()
	{
		if (IsOperatorControl() && IsEnabled())
		{
			//SmartDashboard::PutString("RobotStatus: ", "TestMode START");
			std::cout << "testMode START" << std::endl;
		}
		while (IsTest() && IsEnabled())
		{
			try
			{
				//Put test code here
			}
			catch (std::exception& ex )
			{
				nowError = ex.what();
				if (lastError != nowError)
				{
					std::cout << "[TESTMODE ERROR] " << nowError << std::endl;
					lastError = nowError;
				}
			}
			Wait(kUpdatePeriod);
		}
		std::cout << "testMode STOP" << std::endl;
	}

};

START_ROBOT_CLASS(Robot)