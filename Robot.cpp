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
 *	version 2.14 4/29 - CRM
 *		- remove gyro and score in auto selection
 *	version 2.15 9/17 - CRM
 *		- removed winch code
 *		- added 3 step autonomous movement to score low or high
 *		- added shoot high goal on joystick trigger (button 1)
 *		- add DoNothing selection for auto mode
 *	version 2.16 9/29 - CRM
 *		- fixed bug in autonomous (Drive vs TankDrive again)
 *		- fixed bug in DriveStraight
 *		- changed preferences not to overwrite
 *		- add Servo for kicker on PWM 6
 *		- add kicker on button 11 and 12 to test mode
 *		- add kicker values to smartdashboard
 *	version 2.17 9/29 - CRM
 *		- remove servo
 *		- add pneumatics and solenoid for kicker
 *		- remove autonomous shooting
 *		- add kicker action to lowbar shooting
 *		- add USB camera for high goal view
 *		- put kicker forward during intake
 *	version 2.18 10/4 - CRM
 *		- changed USB camera to cam3
 *		- make sure kicker is forward in autonomous
 *		- add release delay to kicker for intake
 *	version 2.19 10/5 - CRM
 *		- convert to IterativeRobot class
 *
 */
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	VictorSP motorLF, motorLR, motorRF, motorRR;
	VictorSP spinnerLeft, spinnerRight;
	RobotDrive driveTrain;
	RobotDrive driveSpinner;
	Compressor pcm;
	DoubleSolenoid kickSolenoid;
	Joystick xbox, stick;
	PowerDistributionPanel pdp;
	SendableChooser *chooser;
	const std::string autoDoNothing = "DoNothing";
	const std::string autoDriveStraight = "DriveStraight";
	const std::string autoLowBarBackwards = "LowBarBackwards";
	Preferences *prefs;
	uint64_t kickerDelay = 1000; //delay after trigger pull for spinup
	uint64_t releaseDelay = 500; //delay on intake release to retract kicker
	float intakeSpeed = 0.25f;     // speed of motors for intake of ball
	float shootSpeed = 1.0f;       // speed of motors for shooting ball
	float kUpdatePeriod = 0.010f;  // loop delay in milliseconds
	std::string lastError = "";
	std::string nowError = "";
	uint64_t autoTime = 0;
	uint64_t elapsedTime = 0;
	std::string autoSelected = "";

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
	pcm(1),              //pneumatic control module - CAN ID = 1
	kickSolenoid(1,0,1), //double solenoid on PCM channel 0 & 1
	xbox(0),     		 //xbox on driver station USB port 0
	stick(1),    		 //joystick on driver station USB port 1
	pdp(0),				 //power distribution controller - CAN ID = 0
	chooser(),			 //selection on smartdashboard for auto mode
	prefs()				 //preferences values on smartdashboard
	{
	}

	//runs once on robot startup
	void RobotInit()
	{
		//this sets the safety timeout for the drives
		//loops in the functions below must not exceed this timeout value
		driveTrain.SetExpiration(0.50);
		driveSpinner.SetExpiration(0.50);
		motorLF.SetInverted(true);
		motorLR.SetInverted(true);
		motorRF.SetInverted(true);
		motorRR.SetInverted(true);
		spinnerLeft.SetInverted(false);
		spinnerRight.SetInverted(true);

		SmartDashboard::PutString("Code Version","2.19");
		chooser = new SendableChooser();
		chooser->AddDefault(autoDoNothing, (void*)&autoDoNothing);
		chooser->AddObject(autoDriveStraight, (void*)&autoDriveStraight);
		chooser->AddObject(autoLowBarBackwards, (void*)&autoLowBarBackwards);
		SmartDashboard::PutData("Auto Mode", chooser);

		//initialize prefs here
		prefs = Preferences::GetInstance();
		if (!prefs->ContainsKey("shootSpeed"))
			prefs->PutDouble("shootSpeed",1.0);
		if (!prefs->ContainsKey("kickerDelay"))
			prefs->PutLong("kickerDelay",1000);
		if (!prefs->ContainsKey("intakeSpeed"))
			prefs->PutDouble("intakeSpeed",0.25);
		if (!prefs->ContainsKey("releaseDelay"))
			prefs->PutLong("releaseDelay",500);

		pcm.SetClosedLoopControl(true);
		//retract kicker
		kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
		//setup USB camera
		CameraServer::GetInstance()->SetQuality(25);
		CameraServer::GetInstance()->StartAutomaticCapture("cam3");
	}

	//runs once on entering Autonomous mode
	void AutonomousInit()
	{
		driveTrain.SetSafetyEnabled(true);
		driveSpinner.SetSafetyEnabled(true);
		std::cout << "AutoMode START" << std::endl;
		autoSelected = *((std::string*)chooser->GetSelected());
		printf("autoSelected= %s\n",autoSelected.c_str());
		if(autoSelected == autoLowBarBackwards)
		{
			kickSolenoid.Set(DoubleSolenoid::Value::kForward);
		}
		autoTime = GetFPGATime();
	}

	//runs repeatedly while in Autonomous mode
	void AutonomousPeriodic()
	{
		float dsLeft = 0.0f;
		float dsRight = 0.0f;
		float dtLeft = 0.0f;
		float dtRight = 0.0f;

		elapsedTime = GetFPGATime();
		elapsedTime = (elapsedTime - autoTime) / 1000;

		if(autoSelected == autoDriveStraight)
		{
			if (elapsedTime < 2800)  //total travel time
			{
				if (elapsedTime < 1500)
				{
					dtLeft = -0.65;
					dtRight = -0.65;   //drive forward slow
				}
				else
				{
					dtLeft = -1.0;
					dtRight = -1.0;   //drive forward fast
				}
			}
		}
		if(autoSelected == autoLowBarBackwards)
		{
			if (elapsedTime < 2800)  //total travel time
			{
				if (elapsedTime < 1500)
				{
					dtLeft = 0.65;
					dtRight = 0.65;   //drive backward slow
				}
				else
				{
					dtLeft = 1.0;
					dtRight = 1.0;   //drive backward fast
				}
			}
		}

		driveTrain.TankDrive(dtLeft,dtRight);
		driveSpinner.TankDrive(dsLeft,dsRight,false);
	}

	//runs once upon entering Operator Control mode
	void TeleopInit()
	{
		std::cout << "OpMode START" << std::endl;
		driveTrain.SetSafetyEnabled(true);
		driveSpinner.SetSafetyEnabled(true);
		kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
		kickerDelay = prefs->GetLong("kickerDelay");
		shootSpeed = prefs->GetDouble("shootSpeed");
		intakeSpeed = prefs->GetDouble("intakeSpeed");
		releaseDelay = prefs->GetLong("releaseDelay");
	}

	// used in TeleopPeriodic
	void RunShooter()
	{
		static bool triggerflag = false;
		static bool lowballflag = false;
		static bool kickflag = false;
		static bool lowkickflag = false;
		static bool intakeflag = false;
		static bool mankickflag = false;
		static bool releaseflag = false;
		static uint64_t triggerTime = 0;
		static uint64_t lowballTime = 0;
		static uint64_t releaseTime = 0;
		uint64_t elapsedTime = 0;
		float spinLeftVal = 0.0f;     // left side driveSpinner output
		float spinRightVal = 0.0f;	  // right side driveSpinner output


		//run spinner motors to intake ball
		if (stick.GetRawButton(3) && !stick.GetRawButton(4) && !stick.GetRawButton(1))
		{
			if(!intakeflag)
			{
				printf("Intake Button + Kicker\n");
				intakeflag=true;
				//kickSolenoid.Set(DoubleSolenoid::Value::kForward);
			}
			spinLeftVal = -1 * intakeSpeed;
			spinRightVal = intakeSpeed;
		}

		//retract the kicker after delay when intake button released
		if (!stick.GetRawButton(3) && intakeflag)
		{
			if(!releaseflag)
			{
				releaseflag = true;
				releaseTime = GetFPGATime();
			}
			elapsedTime = GetFPGATime();
			elapsedTime = (elapsedTime - releaseTime) / 1000;
			if (elapsedTime > releaseDelay)
			{
				releaseflag = false;
				intakeflag = false;
				//kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
				printf("Intake Kicker Retract");
			}
		}

		//put kicker forward when stick is pushed forward
		if (stick.GetRawAxis(1) < -0.5)
		{
			if(!mankickflag)
			{
				printf("Manual Kick\n");
				mankickflag=true;
				kickSolenoid.Set(DoubleSolenoid::Value::kForward);
			}
		}

		//release kicker when stick is not forward
		if (stick.GetRawAxis(1) > -0.5 && mankickflag)
		{
			mankickflag=false;
			kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
			printf("Manual Kick Retract\n");
		}


		//run spinner motors to shoot ball backward - low goal
		if (stick.GetRawButton(4) && !stick.GetRawButton(3) && !stick.GetRawButton(1))
		{
			if(!lowballflag)
			{
				printf("LowBall Pulled\n");
				lowballflag=true;
				lowballTime = GetFPGATime();
			}
			spinLeftVal = shootSpeed;
			spinRightVal = -1 * shootSpeed;
			elapsedTime = GetFPGATime();
			elapsedTime = (elapsedTime - lowballTime) / 1000;
			//push kicker after delay for shooter wheels spinup
			if (elapsedTime > kickerDelay)
			{
				if(!lowkickflag) {printf("LowBall Kick\n"); lowkickflag=true;}
				kickSolenoid.Set(DoubleSolenoid::Value::kForward);
			}
		}

		//retract the kicker when lowball released
		if (!stick.GetRawButton(4) && lowballflag)
		{
			lowballflag=false;
			lowkickflag=false;
			kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
			printf("LowBall Kicker Retract\n");
		}

		//run spinner motors to shoot ball forward - high goal
		if (stick.GetRawButton(1) && !stick.GetRawButton(3) && !stick.GetRawButton(4))
		{
			if(!triggerflag)
			{
				printf("Trigger Pulled\n");
				triggerflag=true;
				triggerTime = GetFPGATime();
			}
			spinLeftVal = -1 * shootSpeed;
			spinRightVal = shootSpeed;

			elapsedTime = GetFPGATime();
			elapsedTime = (elapsedTime - triggerTime) / 1000;
			//push kicker after delay for shooter wheels spinup
			if (elapsedTime > kickerDelay)
			{
				if(!kickflag) {printf("Trigger Kick\n"); kickflag=true;}
				kickSolenoid.Set(DoubleSolenoid::Value::kForward);
			}
		}

		//retract the kicker when trigger released
		if (!stick.GetRawButton(1) && triggerflag)
		{
			triggerflag=false;
			kickflag=false;
			kickSolenoid.Set(DoubleSolenoid::Value::kReverse);
			printf("Kicker Retract\n");
		}

		driveSpinner.TankDrive(spinLeftVal,spinRightVal,false);
	}

	//used in TeleopPeriodic
	void RunDriveTrain()
	{
		float leftRaw = 0.0f;         // left side joystick input
		float rightRaw = 0.0f;        // right side joystick input

		//get raw joystick values from xbox
		leftRaw = xbox.GetRawAxis(1);
		rightRaw = xbox.GetRawAxis(5);
		//protect from out of range inputs
		if (leftRaw > 0.99) leftRaw = 0.99;
		if (leftRaw <-0.99) leftRaw = -0.99;
		if (rightRaw > 0.99) rightRaw = 0.99;
		if (rightRaw < -0.99) rightRaw = -0.99;
		driveTrain.TankDrive(leftRaw,rightRaw);
	}

	//runs repeatedly while in Operator Control mode
	void TeleopPeriodic()
	{
		RunShooter();
		RunDriveTrain();
	}

	void DisabledInit()
	{
		driveTrain.TankDrive(0.0,0.0);
		driveSpinner.TankDrive(0.0,0.0);
	}

	//runs repeatedly while in Test mode
	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
