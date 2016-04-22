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
 *	version 1.08 3/10/16 - CRM - this was at Rock City Regional
 *		-  remove lagfilter code
 *		-  remove anti-spin limiting code
 *	version 1.09 3/13/16 - CRM
 *		-  remove smartdashboard calls - maybe help camera feed problems
 *	version 1.10 3/27/16 - CRM
 *		-  add optional auto mode for lowbar to low goal score with gyro
 *		-  add ultrasonic rangefinder to look at
 *		-  change error reporting to driverstation instead of console
 *		-  add LiveWindow objects for Test Mode
 *		-  add data to smartdashboard for autonomous and test modes
 *	version 1.11 4/13/16 - CRM
 *	    - fix timeout in autonomous
 *	    - change intake to button 3 - now intake on 3, shoot on 4
 *	    - add tape and winch motors - tape on btn 11, winch on btn 12
 *	    - tape will automatically retract after winch starts
 *	version 2.12 4/16/16 - CRM
 *		- made this a 2.x version with climber
 *		- make sure tape only retracts once per enable
 *	version 2.13 4/21/16 - CRM
 *		- remove automatic retract of tape winch
 */
class Robot: public SampleRobot
{
	VictorSP motorLF, motorLR, motorRF, motorRR;
	VictorSP spinnerLeft, spinnerRight;
	VictorSP winchTape, winchPull;
	RobotDrive driveTrain;
	RobotDrive driveSpinner;
	RobotDrive driveWinch;
	Joystick xbox, stick;
	PowerDistributionPanel pdp;
	ADXRS450_Gyro gyroOne;
	AnalogInput sonar;
	SendableChooser *chooser;
	const std::string autoDriveStraight = "DriveStraight";
	const std::string autoLowBarToLowGoal = "LowBarToLowGoal";
	Preferences *prefs;
	float kUpdatePeriod = 0.015f; // loop delay of 15 milliseconds
	std::string lastError = "";
	std::string nowError = "";
	const float volts_to_inches = 0.0098;
	uint64_t step1Time = 0;
	uint64_t step2Time = 0;
	uint64_t shootTime = 0;
	double step1Speed = 0.0;
	double step2Speed = 0.0;
	double turnSpeed = 0.0;
	double turnAngle = 45.0;
	double tapeSpeed = 0.0;
	double pullSpeed = 0.0;

public:
	Robot() :
		motorLF(0),  		 //Left Front motor on PWM channel 0
		motorLR(1),  		 //Left Rear motor on PWM channel 1
		motorRF(2),  		 //Right Front motor on PWM channel 2
		motorRR(3),  		 //Right Rear motor on PWM channel 3
		spinnerLeft(4), 	 //left spinner motor on PWM channel 4
		spinnerRight(5), 	 //left spinner motor on PWM channel 5
		winchTape(6), 	 	 //tape winch on PWM channel 6
		winchPull(7), 	 	 //pull winch on PWM channel 7
		driveTrain(motorLF,motorLR,motorRF,motorRR),
		driveSpinner(spinnerLeft,spinnerRight),
		driveWinch(winchTape,winchPull),
		xbox(0),     		 //xbox on driver station USB port 0
		stick(1),    		 //joystick on driver station USB port 1
		pdp(),				 //power distribution controller - CAN ID = 0
		gyroOne(SPI::Port::kOnboardCS0),	 //gyro on SPI port 0
		sonar(0),
		chooser(),
	    prefs()
	{
	}

	void RobotInit()
	{
		//this sets the safety timeout for the driveTrain motors
		//loops in the functions below must not exceed this timeout value
		motorLF.SetInverted(false);
		motorLR.SetInverted(false);
		motorRF.SetInverted(false);
		motorRR.SetInverted(false);
		spinnerLeft.SetInverted(false);
		spinnerRight.SetInverted(false);
		driveTrain.SetExpiration(0.25);
		driveSpinner.SetExpiration(0.25);
		driveWinch.SetExpiration(0.25);
		SmartDashboard::PutString("Code Version","2.13");
		chooser = new SendableChooser();
		chooser->AddDefault(autoDriveStraight, (void*)&autoDriveStraight);
		chooser->AddObject(autoLowBarToLowGoal, (void*)&autoLowBarToLowGoal);
		SmartDashboard::PutData("Auto Mode", chooser);
		//initialize prefs here
		prefs = Preferences::GetInstance();
		if (!prefs->ContainsKey("autoStep1Time"))
			prefs->PutLong("autoStep1Time",1500);
		if (!prefs->ContainsKey("autoStep2Time"))
			prefs->PutLong("autoStep2Time",1500);
		if (!prefs->ContainsKey("autoShootTime"))
			prefs->PutLong("autoShootTime",1500);
		if (!prefs->ContainsKey("autoStep1Speed"))
			prefs->PutDouble("autoStep1Speed",-0.65);
		if (!prefs->ContainsKey("autoStep2Speed"))
			prefs->PutDouble("autoStep2Speed",-0.65);
		if (!prefs->ContainsKey("autoTurnSpeed"))
			prefs->PutDouble("autoTurnSpeed",-0.65);
		if (!prefs->ContainsKey("autoTurnAngle"))
			prefs->PutDouble("autoTurnAngle",45.0);
		if (!prefs->ContainsKey("tapeSpeed"))
			prefs->PutDouble("tapeSpeed",1.00);
		if (!prefs->ContainsKey("pullSpeed"))
			prefs->PutDouble("pullSpeed",1.00);
		prefs->Save();
	}

	void Autonomous()
		{
			uint64_t autoTime = 0;
			uint64_t testTime = 0;
			uint64_t totalTime = 0;
			float dsLeft = 0.0f;
			float dsRight = 0.0f;
			float dtLeft = 0.0f;
			float dtRight = 0.0f;
			float bearing = 0.0f;         //direction we want to go
			float heading = 0.0f;		  //direction we are going
			float voltage(0);
			float distance(0);
			std::string autoSelected = "";

			if (IsAutonomous() && IsEnabled())
			{
				driveTrain.SetSafetyEnabled(true);
				driveSpinner.SetSafetyEnabled(true);
				driveWinch.SetSafetyEnabled(false);
				step1Time = prefs->GetLong("autoStep1Time");
				step2Time = prefs->GetLong("autoStep2Time");
				shootTime = prefs->GetLong("autoShootTime");
				totalTime = step1Time + step2Time;
				step1Speed = prefs->GetDouble("autoStep1Speed");
				step2Speed = prefs->GetDouble("autoStep2Speed");
				turnSpeed = prefs->GetDouble("autoTurnSpeed");
				turnAngle = prefs->GetDouble("autoTurnAngle");
				if (turnAngle == 0) turnAngle = 45.0;
				autoTime = GetFPGATime();
				bearing = gyroOne.GetAngle() + turnAngle;
				SmartDashboard::PutNumber("Bearing",bearing);
				autoSelected = *((std::string*)chooser->GetSelected());
			}
			while (IsAutonomous() && IsEnabled())
			{
				dtLeft = 0.0;
				dtRight = 0.0;
				dsLeft = 0.0;
				dsRight = 0.0;
				if(autoSelected == autoLowBarToLowGoal)
				{
					voltage = sonar.GetAverageVoltage();
					distance = voltage / volts_to_inches;
					testTime = GetFPGATime();
					testTime = (testTime - autoTime) / 1000;  //get milliseconds
					heading = gyroOne.GetAngle();
					if (testTime < totalTime)  //total travel time
					{
						if (testTime < step1Time)
						{
							dtLeft = step1Speed;
							dtRight = heading;   //drive straight past lowbar
						}
						else
						{
							if (heading < bearing)
							{
								dtLeft = turnSpeed;
								dtRight = (bearing - heading)/turnAngle;  //curve to new heading
							}
							else
							{
								dtLeft = step2Speed;
								dtRight = heading; //drive straight to low goal
							}
						}
					}
					if (testTime > totalTime && testTime < totalTime + shootTime)
					{
						dsLeft = 1.0;
						dsRight = -1.0;
					}
					SmartDashboard::PutNumber("Heading", heading);
					SmartDashboard::PutNumber("Distance",distance);
				}
				if(autoSelected == autoDriveStraight)
				{
					testTime = GetFPGATime();
					testTime = (testTime - autoTime) / 1000;  //get milliseconds
					if (testTime < 2800)  //total travel time
					{
						if (testTime < 1500)
						{
							dtLeft = -0.65;
							dtRight = -0.65;   //drive forward
						}
						else
						{
							dtLeft = -1.0;
							dtRight = -1.0;   //drive forward
						}
					}
				}
				driveTrain.Drive(dtLeft,dtRight);
				driveSpinner.TankDrive(dsLeft,dsRight,false);
				Wait(kUpdatePeriod); // Wait delay time before looping
			}
			std::cout << "AutoMode STOP" << std::endl;
		}

	void RunShooter()
	{
		float spinLeftVal = 0.0f;     // left side driveSpinner output
		float spinRightVal = 0.0f;	  // right side driveSpinner output
		float intakeSpeed = 0.25f;    // speed of motors for intake of ball
		float shootSpeed = 1.0f;      // speed of motors for shooting ball

		try
		{
			spinLeftVal = 0;
			spinRightVal = 0;
			//run spinner motors to intake ball
			if (stick.GetRawButton(3) == true && stick.GetRawButton(4) == false)
			{
				spinLeftVal = -1 * intakeSpeed;
				spinRightVal = intakeSpeed;
			}

			//run spinner motors to shoot ball backward - low goal
			if (stick.GetRawButton(4) == true && stick.GetRawButton(3) == false)
			{
				spinLeftVal = shootSpeed;
				spinRightVal = -1 * shootSpeed;
			}

			//run spinner motors to shoot ball forward - high goal
			//hold trigger and button 4 for high goal
			if (stick.GetRawButton(4) == true && stick.GetRawButton(0) == true)
			{
				spinLeftVal = -1 * shootSpeed;
				spinRightVal = shootSpeed;
			}

			driveSpinner.TankDrive(spinLeftVal,spinRightVal,false);
		}
		catch (std::exception& ex )
		{
			nowError = ex.what();
			if (lastError != nowError)
			{
				DriverStation::ReportError("[SHOOTER ERROR] " + nowError + "\n");
				lastError = nowError;
			}
		}
	}

	void RunDriveTrain()
	{
		float leftRaw = 0.0f;         // left side joystick input
		float leftVal = 0.0f;		  // left side drivetrain output
		float rightRaw = 0.0f;        // right side joystick input
		float rightVal = 0.0f;		  // right side drivetrain output

		try
		{
			//get raw joystick values from xbox
			leftRaw = xbox.GetRawAxis(1);
			rightRaw = xbox.GetRawAxis(5);
			//if filtering needed - put it here
			leftVal = leftRaw;
			rightVal = rightRaw;
			//protect from out of range inputs
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
				DriverStation::ReportError("[DRIVETRAIN ERROR] " + nowError + "\n");
				lastError = nowError;
			}
		}
	}

	void OperatorControl()
	{
		float winchTapeVal = 0.0f;
		float winchPullVal = 0.0f;

		if (IsOperatorControl() && IsEnabled())
		{
			std::cout << "OpMode START" << std::endl;
			driveTrain.SetExpiration(0.25);
			driveSpinner.SetExpiration(0.25);
			driveWinch.SetExpiration(0.25);
			driveTrain.SetSafetyEnabled(true);
			driveSpinner.SetSafetyEnabled(true);
			driveWinch.SetSafetyEnabled(true);
			tapeSpeed = prefs->GetDouble("tapeSpeed",1.00);
			pullSpeed = prefs->GetDouble("pullSpeed",1.00);
		}
		while (IsOperatorControl() && IsEnabled())
		{
			try
			{
				RunShooter();
				RunDriveTrain();
				winchTapeVal = 0;
				winchPullVal = 0;
				//pull in on tape winch
				if (stick.GetRawButton(9) == true  && stick.GetRawButton(11) == false)
					winchTapeVal = -1 * tapeSpeed;
				//pay out on tape winch
				if (stick.GetRawButton(11) == true  && stick.GetRawButton(9) == false)
					winchTapeVal = tapeSpeed;
				//pull in on pull winch
				if (stick.GetRawButton(12) == true && stick.GetRawButton(11) == false)
					winchPullVal = -1 * pullSpeed;
				//pay out on pull winch
				if (stick.GetRawButton(10) == true && stick.GetRawButton(12) == false)
					winchPullVal = pullSpeed;
				driveWinch.TankDrive(winchTapeVal,winchPullVal,false);
			}
			catch (std::exception& ex )
			{
				nowError = ex.what();
				if (lastError != nowError)
				{
					DriverStation::ReportError("[OPMODE ERROR] " + nowError + "\n");
					lastError = nowError;
				}
			}
			Wait(kUpdatePeriod);
		}
		driveTrain.TankDrive(0.0,0.0,false);
		driveSpinner.TankDrive(0.0,0.0,false);
		driveWinch.TankDrive(0.0,0.0,false);
		std::cout << "OpMode STOP" << std::endl;
	}

	void Test()
	{
		float winchTapeVal = 0.0f;
		float winchPullVal = 0.0f;

		if (IsTest() && IsEnabled())
		{
			std::cout << "testMode START" << std::endl;
			//LiveWindow::GetInstance()->Run();
		}
		while (IsTest() && IsEnabled())
		{
			try
			{
				//pull in on tape winch
				if (stick.GetRawButton(9) == true  && stick.GetRawButton(11) == false)
					winchTapeVal = -1 * tapeSpeed;
				//pay out on tape winch
				if (stick.GetRawButton(11) == true  && stick.GetRawButton(9) == false)
					winchTapeVal = tapeSpeed;
				//pay out on pull winch
				if (stick.GetRawButton(10) == true && stick.GetRawButton(12) == false)
					winchPullVal = pullSpeed;
				//pull in on pull winch
				if (stick.GetRawButton(12) == true && stick.GetRawButton(11) == false)
					winchPullVal = -1 * pullSpeed;
				driveWinch.TankDrive(winchTapeVal,winchPullVal,false);
				driveTrain.TankDrive(0.0,0.0,false);
				driveSpinner.TankDrive(0.0,0.0,false);
				SmartDashboard::PutNumber("Heading", (double)gyroOne.GetAngle());
				SmartDashboard::PutNumber("Distance",sonar.GetAverageVoltage()/volts_to_inches);
			}
			catch (std::exception& ex )
			{
				nowError = ex.what();
				if (lastError != nowError)
				{
					DriverStation::ReportError("[TESTMODE ERROR] " + nowError + "\n");
					lastError = nowError;
				}
			}
			Wait(kUpdatePeriod);
		}
		std::cout << "testMode STOP" << std::endl;
	}

};

START_ROBOT_CLASS(Robot)
