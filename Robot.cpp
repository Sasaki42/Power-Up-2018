//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2018 POWER UP ROBOT CODE
/*
 * Veneer F.M.
 */
//------------------------------------------------------------------------------


#include <iostream>
#include <string>
#include "WPILib.h"
#include <cmath>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DriverStation.h>
#include "../Climber.h"
#include "../Intake.h"
#include "../Elevator.h"


#define CONSOLE

class PowerUpRobot : public IterativeRobot {

//--------------------------------------------------------------------------
// DECLARATION OF PUBLIC METHODS
//--------------------------------------------------------------------------
public:

	// Constructor and destructor methods
	PowerUpRobot();
	~PowerUpRobot();

		//----------------------------------------------------------------------
		// OVERRIDES TO IterativeRobot BASE CLASS VIRTUAL METHODS
		//----------------------------------------------------------------------
		// Robot and state initialization methods
		void   RobotInit();
		void   DisabledInit();
		void   AutonomousInit();
		void   TeleopInit();

		// Robot periodic methods performed in loops
		void   DisabledPeriodic();
		void   AutonomousPeriodic();
		void   TeleopPeriodic();



private: //here goes all the inputs for the ports and switches and stuff
	//----------------------------------------------------------------------
	// CONSTANTS USED IN CLASS
	//----------------------------------------------------------------------
	// DRIVER STATION PORTS AND CHANNELS
	//----------------------------------------------------------------------
	// Driver Station Joystick ports
	static const uint JS_PORT_LEFT           =  0;
	static const uint JS_PORT_RIGHT			 =  1;
	//static const uint JS_PORT_ELV1		 =  1;//Y1
	//static const uint JS_PORT_ELV2 		 =  2;//Y5
	//static const uint CCI_PORT_IN			 =  1;
	//static const uint CCI_PORT_OUT		 =  3;
	//static const uint CCI_PORT_UP			 =  5;
	//static const uint CCI_PORT_DOWN        =  6;
	static const uint CCI_PORT1        	     =  2;  // eStop Robots CCI Inputs

	// Joystick Button (right joystick)
	static const uint CAMERA_LIGHT_SW_CH     =  1;

	// Driver Station CCI1 Channels (Uses joystick button references)
	// using x-box controller
		static const uint Intake_SW_CH			 =  6;
		static const uint Outtake_SW_CH			 =  5;
		static const uint AIROUT_SW_CH 			 =  1;
		static const uint AIRIN_SW_CH			 =  2;

		//static const uint CLIMB_SW_CH    		 =  9;
		//static const uint LOWER_SW_CH			 =  10;

		//static const uint Elv1Up_SW_CH 		 =  4;
		//static const uint Elv1Down_SW_CH		 =  3;
		//static const uint Elv2Up_SW_CH 		 =  2;
		//static const uint Elv2Down_SW_CH		 =  1;

	// Driver Station CCI2 Channels (Uses joystick button references)
	//----------------------------------------------------------------------
	// ROBOT CHANNELS - INPUTS AND OUTPUTS
	//----------------------------------------------------------------------
    // ROBOT INPUTS
	//----------------------------------------------------------------------
	// roboRio GPIO Channels
	static const uint AUTONOMOUS_SW_1_CH        =  0;
	static const uint AUTONOMOUS_SW_2_CH        =  1;
	static const uint AUTONOMOUS_SW_3_CH        =  2;

	// roboRio Analog Channels

	// roboRio Relay Channels
	static const uint CAMERA_LIGHT_CH           =  0;
	//----------------------------------------------------------------------
    // ROBOT OUTPUTS
	//----------------------------------------------------------------------

	// roboRio PWM Channels (PWM = Pulse width modulation)

	//remap these:
		static const uint RIGHT_FRONT_MOTOR_CH	  =  0;
		static const uint RIGHT_REAR_MOTOR_CH     =  1;
		static const uint LEFT_FRONT_MOTOR_CH	  =  3;
		static const uint LEFT_REAR_MOTOR_CH	  =  2;
		static const uint LEFT_INTAKE_MOTOR_CH	  =  6;
		static const uint RIGHT_INTAKE_MOTOR_CH	  =  7;
		static const uint CLIMBER_MOTOR1_CH		  =  4;
		static const uint CLIMBER_MOTOR2_CH		  =  5;
		static const uint ELEVATOR_MOTOR1_CH 	  =  8;
		static const uint ELEVATOR_MOTOR2_CH	  =  9;


	// roboRio Relay Channels


	// roboRio Solenoid Channels
		static const uint Compressor_CH 		  =  0;
		static const uint Solenoid1a_CH			  =  0;
		static const uint Solenoid2a_CH	   		  =  1;
		static const uint Solenoid1b_CH			  =  2;
		static const uint Solenoid2b_CH	   		  =  3;
		static const uint Solenoid1c_CH			  =  4;
		static const uint Solenoid2c_CH	   		  =  5;

	//----------------------------------------------------------------------
	// AUTONOMOUS MODE ROBOT CONTROL CONSTANTS (OUTPUTS)
		// if (robot != working){
		// robot = working;)
	//----------------------------------------------------------------------
    // Autonomous mode timings (Starts and Durations)

	// For durations, 50 loops is approximately 1 second.
	//all the timings and stuff

        // Auto Drive speed
		const float Speed_Drive_Auto    	      =	 0.5;
		const float Speed_Elevator_Auto			  =  0.5;
		// Auto Drive Time
		static const uint Time_Foward_Side1	      =  200;
		static const uint Time_Foward_Side2	      =  300;
		static const uint Time_Foward_ToNull  	  =  500;
		static const uint Time_Foward_Center      =  200;
		static const uint Time_Turn_Center	      =  210;
		static const uint Time_Turn_Side          =  230;
		static const uint Time_Elevator		  	  =  250;
		// Delay value
		static const uint Time_Elevator_Delay	  =  50;
		bool			   Delayed			      = false;
		bool 			   DelayTriggered 		  = false;

    //----------------------------------------------------------------------
	// AUTONOMOUS MODE ROBOT STATE & TIMING TRACKING
	// Used to determine what robot is or should be doing in autonomous mode
	//----------------------------------------------------------------------
    // Autonomous Mode States
	enum autoModeStates {kAM1Off
	//						   ,kAM2DriveLowBar, kAM3DriveOnly
	//        				   , kAM4TurnLowBar, kAM5LowBarShootLow
	//						   , kAM6LowBarShootHigh, kAM7LowBarShootLow2
	//        				   , kAM8LowBarShootHigh2
							   };
//Autonomous variables
	std::string gameData;
	//field Starting_Position
	int Starting_Position;

	//  sides of field objects
	//----------------------------------------------------------------------
	// POINTERS FOR REQUIRED OBJECTS
	//----------------------------------------------------------------------
	// DRIVER STATION INPUT & OUTPUT POINTERS
	//----------------------------------------------------------------------
	// Includes driver station laptop, joysticks, switches and other digital
	// and analog devices connected through the eStop Robotics CCI.
	//----------------------------------------------------------------------
	Joystick		 *pDriveStickLeft;
	Joystick		 *pDriveStickRight;
	//Joystick		   *pDriveStickElv1;
	//Joystick         *pDriveStickElv2;
	Joystick         *pCCI1;             // CCI

	//Joystick Buttons - Right Joystick
	JoystickButton   *pCameraLightButton;

	// eStop Robotics Custom Control Interface (CCI) #1
	//switchbox i think

	// eStop Robotics Custom Control Interface (CCI) #2
	//other switchbox maybe?

    //JoystickButton   *pClimberSwitch;
    //JoystickButton   *pLowerSwitch;
    JoystickButton 	 *pIntakeButton;
    JoystickButton   *pOuttakeButton;
    JoystickButton   *pAirOut;
    JoystickButton   *pAirIn;
    //JoystickButton   *pElv1Up;
    //JoystickButton   *pElv1Down;
    //JoystickButton   *pElv2Up;
    //JoystickButton   *pElv2Down;

	//----------------------------------------------------------------------
	// ROBOT INPUT & OUTPUT POINTERS
	//----------------------------------------------------------------------
	// Robot Digital Inputs - GPIO Inputs including Encoders
	//----------------------------------------------------------------------
	// Autonomous Mode Switches
    DigitalInput    *pAutoSwitch1;
    DigitalInput    *pAutoSwitch2;
    DigitalInput    *pAutoSwitch3;
	//----------------------------------------------------------------------
	// Robot Digital Outputs - Relays (Spikes)
	//----------------------------------------------------------------------

	//----------------------------------------------------------------------
	// Robot Objects
	//----------------------------------------------------------------------
    //the points for all the mechanisms
	RobotDrive		*pDriveTrain;
	Climber			*pClimber;
	Intake 			*pIntake;
	Elevator 		*pElevator;
	//pneumatics
	Compressor 		*pCompressor;

	frc::DoubleSolenoid Solenoid1  {Solenoid1a_CH, Solenoid2a_CH};
	frc::DoubleSolenoid Solenoid2 {Solenoid1b_CH, Solenoid2b_CH};
	frc::DoubleSolenoid Solenoid3 {Solenoid1c_CH, Solenoid2c_CH};
	//----------------------------------------------------------------------
	// CLASS VARIABLES USED TO TRACK ROBOT STATUS
	//----------------------------------------------------------------------
	// General status tracking
	//----------------------------------------------------------------------
	// Class variables to track look (packet) counts
	uint   loopCount;
	uint   DelayCounter;


	//here goes variables for mechanisms


	//----------------------------------------------------------------------
	// Autonomous Mode Timings
	//----------------------------------------------------------------------
	//for resetting auto?


	//----------------------------------------------------------------------
	// CUSTOM METHODS SPECIFIC TO TEAM 1280 ROBOT
	//----------------------------------------------------------------------
	// Initialization and reset methods

	// Driver station and robot input gathering methods
	void   GetDriverStationInput();
	void   ShowDSValues();
	void   GetRobotSensorInput();
	void   ShowRobotValues();

	// Robot output methods
	//functions that instruct mechanisms

	// Autonomous mode methods
	void   CalcAutoModeTimings();
	void   GetAutoModeSwitches();
	void   RunAutonomousMode();
	void   ShowAMStatus();
	void   AM1Off();
	void   AMDriveFwd();
	void   RunClimber();
	void   RunIntake();
	void   RunElevator();
	void   AutoCubeSide();
	void   AutoCubeCenter();
	void   AutoSwitch();
	//void   SolenoidState(bool state);
	void   SolenoidUpdate();
	void   AirIn();
	void   AirOut();
	void   Stage1Stop();
	void   Stage2Stop();
	void   Stage1Start();
	void   Stage2Start();
	void   StageUpdate();
	void   Delay();
	bool   FMSFetch();
	//----------------------------------------------------------------------
	// CLASS VARIABLES USED TO TRACK ROBOT STATUS
	//----------------------------------------------------------------------
	// General status tracking
	//----------------------------------------------------------------------


	//Drive Train
	float rightDriveSpeed;
	float leftDriveSpeed;
	//float Elv1DriveSpeed;
	//float Elv2DriveSpeed;

	float LeftY;
	float RightY;
	float LeftTrigger;
	float RightTrigger;


	const double period =0.0;



/*	void PowerUpRobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

*/
/*


	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.

	void AutonomousInit() override {
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

*/
};

START_ROBOT_CLASS(PowerUpRobot)

//------------------------------------------------------------------------------
// METHOD DEFINITONS
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::PowerUpRobot
// Type:	Public default constructor for PowerUpRobot class
//------------------------------------------------------------------------------
// Defines pointers to required robot objects and initializes packet count
// variable.
//------------------------------------------------------------------------------

PowerUpRobot::PowerUpRobot()
{
	//----------------------------------------------------------------------
	// DEFINE POINTERS TO REQUIRED ROBOT OBJECTS
	//----------------------------------------------------------------------
	// DRIVER STATION INPUTS
	//----------------------------------------------------------------------
	// Driver Station Object

	// Define joysticks & CCI
	pDriveStickLeft		   = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	   = new Joystick(JS_PORT_RIGHT);
	//pDriveStickElv1		   = new Joystick(JS_PORT_ELV1);
	//pDriveStickElevator    = new Joystick(JS_PORT_ELEVATOR);
	pCCI1                  = new Joystick(CCI_PORT1); // CCI uses joystick object


	// Joystick Buttons (Right Joystick)
	pCameraLightButton     = new JoystickButton(pDriveStickRight,CAMERA_LIGHT_SW_CH);
	// Driver Station locations
	//ds 					   = new DriverStation();
	//CCI 2 Switches
	//pClimberSwitch         = new JoystickButton(pCCI1,CLIMB_SW_CH);
	//pLowerSwitch 		   = new JoystickButton(pCCI1, LOWER_SW_CH);
	pIntakeButton 		   = new JoystickButton(pCCI1,Intake_SW_CH);
	pOuttakeButton 		   = new JoystickButton(pCCI1,Outtake_SW_CH);
	//pElv1Up				   = new JoystickButton(pCCI1,Elv1Up_SW_CH);
	//pElv1Down    		   = new JoystickButton(pCCI1,Elv1Down_SW_CH);
	//pElv2Up				   = new JoystickButton(pCCI1,Elv2Up_SW_CH);
	//pElv2Down    		   = new JoystickButton(pCCI1,Elv2Down_SW_CH);
	pAirOut				   = new JoystickButton(pCCI1,AIRIN_SW_CH);
	pAirIn				   = new JoystickButton(pCCI1,AIROUT_SW_CH);
	/*
	pIntakeButton 		   = new JoystickButton(pDriverStickIntak,Intake_SW_CH);
	pOuttakeButton 		   = new JoystickButton(pDriverStickOut, Outtake_SW_CH);
	pElvOn				   = new JoystickButton(pDriverStickUp,Elevator_Motor1_CH);
	pElvDown    		   = new JoystickButton(pDriverStickDown, Elevator_Motor2_CH);
	*/
	//pnematics
	pCompressor 		   = new Compressor(Compressor_CH);
	//ds = new DriverStation();
	//----------------------------------------------------------------------
	// ROBOT INPUTS
	//----------------------------------------------------------------------
	// GPIO & Spare Power Inputs
	// - Autonomous Mode Switches
	pAutoSwitch1         = new DigitalInput(AUTONOMOUS_SW_1_CH);
	pAutoSwitch2         = new DigitalInput(AUTONOMOUS_SW_2_CH);
	pAutoSwitch3         = new DigitalInput(AUTONOMOUS_SW_3_CH);

	//----------------------------------------------------------------------
	// ROBOT CONTROLS (OUTPUTS)
	//----------------------------------------------------------------------
	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
							RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);
	pClimber		     = new Climber(CLIMBER_MOTOR1_CH, CLIMBER_MOTOR2_CH);
	pIntake 			 = new Intake(LEFT_INTAKE_MOTOR_CH, RIGHT_INTAKE_MOTOR_CH);
	pElevator 			 = new Elevator(ELEVATOR_MOTOR1_CH, ELEVATOR_MOTOR2_CH);
	//----------------------------------------------------------------------
	// INITIALIZE VARIABLES
	//----------------------------------------------------------------------
	// Initialize loop counter
	loopCount      		  = 0;
	DelayCounter			  = 0;

	// Initialize robot control variables
//	autoMode              = kAM1Off;
	rightDriveSpeed       = 0.0;
	leftDriveSpeed        = 0.0;
	//Elv1DriveSpeed        = 0.0;
	//Elv2DriveSpeed        = 0.0;

	LeftY				  = 0.0;
	RightY				  = 0.0;
	LeftTrigger  		  = 0.0;
	RightTrigger		  = 0.0;



// Robot field Starting_Position
	Starting_Position     = 1;

	//current = pCompressor->GetCompressorCurrent();
	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::~PowerUpRobot
// Type:	Public default destructor for PowerUpRobot class
//------------------------------------------------------------------------------
PowerUpRobot::~PowerUpRobot()
{
}

//------------------------------------------------------------------------------
// METHOD:  SteamWorksRobot::RobotInit()
// Type:	Performs robot initiation functions.  Overrides RobotInit() virtual
//          method contained in WPILib.
//
//			These actions are performed once and only once when the robot is
//	        powered on.
//------------------------------------------------------------------------------
// Functions:
// - Initializes the SmartDashboard
//------------------------------------------------------------------------------
void PowerUpRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif

	//variables for smartdashboard i think

}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::DisabledInit()
// Type:	Executes when the robot is first placed in Disabled mode.  Overrides
//			DisabledInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets loop counter for disabled mode
//------------------------------------------------------------------------------
void PowerUpRobot::DisabledInit()
{
	// Reset loop counter
	loopCount  = 0;
	//SolenoidState(true);
	AirOut();
	return;
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::AutonomousInit()
// Type:	Executes when the robot is first placed in Autonomous mode.
//			Overrides AutonomousInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counter for autonomous mode
// - Determines which autonomous mode we want to use
// - Optionally displays the status of the autonomous mode switches for debugging
//   purposes
//--------------------------------------------------- ---------------------------
void PowerUpRobot::AutonomousInit()
{
	// Reset loop counter
	loopCount  = 0;
	//AirOut solenoids when robot is initialized:
	//SolenoidState(false);
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	CalcAutoModeTimings();
//	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();
//	pCameraLight->TurnOff();
	//Starting_Position= ds->GetLocation();
	Starting_Position = frc::DriverStation::GetInstance().GetLocation(); // get the starting Starting_Position
	//gameData= frc::DriverStation::GetInstance().GetGameSpecificMessage();  // get sides of switches, scale
	gameData = "L";

	/*all 3.14 lines of code Tyler wrote this year:

	MemezVirus.Run();
	//bugfixing
	if (code != work){
		code = fixed;
	}

	*/

	//If not connected to fms returns out-of-bounds values
	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::TeleopInit()
// Type:	Executes when the robot is first placed in Teleoperated mode.
//			Overrides TeleopInit() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Resets the loop counters for teleoperated mode
// - Sets the ator arm in Starting_Position variable to true so arm doesn't move
//------------------------------------------------------------------------------
void PowerUpRobot::TeleopInit()
{
	// Loop count initialization
	loopCount      = 0;
	DelayCounter   = 0;
	//SolenoidState(true);
	//AirOut();
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	//Stage1Start();
	//setting bools and the like to whatever it needs to be
	//Compressor start:
	pCompressor->SetClosedLoopControl(true);
	return;
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::DisabledPeriodic()
// Type:	Executes when the robot is in disabled mode.  Overrides the
//			DisabledPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increment the disabled loop counter
// - Optionally displays robot inputs (autonomous mode switches and sensors)
//------------------------------------------------------------------------------
void PowerUpRobot::DisabledPeriodic()
{
    // Increment loop counter
	loopCount++;
	//SolenoidState(false);
	AirOut();
	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();

#ifdef CONSOLE
	ShowRobotValues();
#endif

	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::AutonomousPeriodic()
// Type:	Executes when the robot is in autonomous ode.  Overrides the
//			AutonomousPeriodic() virtual method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops while in autonomous mode.
// - Gets robot sensor inputs
// - Runs autonomous mode
//------------------------------------------------------------------------------
void PowerUpRobot::AutonomousPeriodic()
{
	if(FMSFetch()){
    // Increment & display loop counter
	loopCount++;
	DelayCounter++;
//	GetAutoModeSwitches();
	GetRobotSensorInput();
	ShowAMStatus();
	RunAutonomousMode();
	AutoSwitch();
	SmartDashboard::PutNumber("LoopCount",loopCount);
	}
	else{
		Starting_Position = frc::DriverStation::GetInstance().GetLocation(); // get the starting Starting_Position
		//Starting_Position = 1;
		gameData= frc::DriverStation::GetInstance().GetGameSpecificMessage();  // get sides of switches, scale
		//gameData="L";
	}
	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::TeleopPeriodic()
// Type:	Executes whqen the robot is in teleoperated mode each time a new
//          packet of information has been received by the Driver Station.  Any
//          code which needs new information from the Driver Station should be
//          placed here.  This method overrides the TeleopPeriodic() virtual
//          method contained in WPILib.
//------------------------------------------------------------------------------
// Functions:
// - Increments the count of loops processed and packets received while
//   in teleoperated mode.
// - Obtains input from the driver station (joystick inputs, switches,
//   potentiometers, etc.)
// - Obtains inputs from the robot (analog and digital sensors)
// - Moves the arm ator to the target Starting_Position based on driver station
//   inputs
// - Sets the drive motor values based on joystick movement
//------------------------------------------------------------------------------
void PowerUpRobot::TeleopPeriodic()
{
	ShowAMStatus();
	// Get inputs from the driver station
		GetDriverStationInput();
		// Get robot sensor input
		GetRobotSensorInput();
	    // Determine if climber should run
	// Increment & display loop counter
	loopCount++;
	DelayCounter++;

    RunClimber();
    RunIntake();
    RunElevator();
    SolenoidUpdate();
	// Drive Robot using Tank Drive
	pDriveTrain->TankDrive(-leftDriveSpeed,-rightDriveSpeed);
	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::GetDriverStationInput()
// Type:	Public accessor for SteamWorksRobot class
//hey there person this is a comment for you :)
//------------------------------------------------------------------------------
// Obtains the input from the DriverStation required for teleoperated mode.
// Includes obtaining input for the following switches:
// - Optionally displays driver station values
//------------------------------------------------------------------------------
void PowerUpRobot::GetDriverStationInput()
{

	rightDriveSpeed		=  pDriveStickRight->GetY();
	leftDriveSpeed		=  pDriveStickLeft->GetY();
	//Elv1DriveSpeed		=  pCCI1->GetY();
	//Elv2DriveSpeed		=  pCCI1->GetY();

	LeftY               = pCCI1->GetRawAxis(1);
	RightY              = pCCI1->GetRawAxis(5);
	LeftTrigger         = pCCI1->GetRawAxis(2);
	RightTrigger        = pCCI1->GetRawAxis(3);

	//maybe 1 for left, 5 for right, and 2 and 3 for triggers, though everything else says otherwise
	//so probably not, putting it here just in case

	//get things for buttons to do stuff in teleop here too

#ifdef CONSOLE
    ShowDSValues();
#endif

	return;
}
#ifdef CONSOLE

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::ShowDSValues()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
// Displays desired driver station values to the SmartDashboard for display
// purposes.
//------------------------------------------------------------------------------
void PowerUpRobot::ShowDSValues()
{
	// Show the values for driver station inputs
	SmartDashboard::PutNumber ("Joystick Right",pDriveStickRight->GetY());
	SmartDashboard::PutNumber ("Joystick Left",pDriveStickLeft->GetY());
	//SmartDashboard::PutBoolean("DS Raise Robot",pClimberSwitch->Get());
	//SmartDashboard::PutBoolean("DS Lower Robot",pLowerSwitch->Get());
	SmartDashboard::PutBoolean("DS Intake Robot",pIntakeButton->Get());
	SmartDashboard::PutBoolean("DS Air Out    ",pAirOut->Get());
	SmartDashboard::PutBoolean("DS Air In     ",pAirIn->Get());
	// add 1 other Smart Dashboard
	return;
}
#endif

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::GetRobotSensorInput()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
// Looks at data in the packet and obtains input coming from the robot to be
// used in both Autonomous and Teleoperated Modes.
// - Distance traveled by right wheels
// - Distance traveled by left wheels
//------------------------------------------------------------------------------
void PowerUpRobot::GetRobotSensorInput()
{
#ifdef CONSOLE
	ShowRobotValues();
#endif

	return;
}
#ifdef CONSOLE

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::ShowRobotValues()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PowerUpRobot::ShowRobotValues()
{
//	SmartDashboard::PutNumber("AM Mode",autoMode);
	SmartDashboard::PutBoolean("R AM Switch 1",pAutoSwitch1->Get());
	SmartDashboard::PutBoolean("R AM Switch 2",pAutoSwitch2->Get());
	SmartDashboard::PutBoolean("R AM Switch 3",pAutoSwitch3->Get());
	return;
}
#endif


//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::RunClimber()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void PowerUpRobot::AutoSwitch(){
	switch(Starting_Position){
	//so apparently the gamedata doesn't show up for a little while, this might
	//have to be put into a loop to try to detect it for a few loops
								case 1: 		//LEFT Side
								if(gameData[0]=='L'){
									//put cube on switch
									AutoCubeSide();
								}
								else if(gameData[0]=='R'){
								// go to null
									if (loopCount < Time_Foward_ToNull) {//insert time
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else{
										//nothing
									}
								}
								else{
								//ERROR
								}
								break;
							case 2: //center field
								if(gameData[0]=='L'){
									// go left
									if (loopCount < Time_Turn_Center) {//Turning. insert time
									pDriveTrain->TankDrive(-Speed_Drive_Auto,Speed_Drive_Auto);// L<0, R>0
									}
									else if(loopCount < Time_Foward_Center){//going forward
									}
									else{
									AutoCubeCenter();
									}
									}
								else if(gameData[0]=='R'){
									// go right
									if (loopCount < Time_Turn_Center) {//Turning. insert time
									pDriveTrain->TankDrive(Speed_Drive_Auto,-Speed_Drive_Auto);// L>0, R<0
									}
									else if(loopCount < Time_Foward_Center){//going forward
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);// insert speed
									}
									else{
									AutoCubeCenter();
									}
								}
								else{
								//ERROR
								}
								break;
							case 3: 	     	//RIGHT Side
								if(gameData[0]=='L'){
									//go to null
									if (loopCount < Time_Foward_ToNull) {//insert time
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else{
									//nothing
									}
									}
									else if(gameData[0]=='R'){
									//put cube on switch
										AutoCubeSide();
									}
									else{
									//ERROR
									}
									break;
								}
	return;
}



void PowerUpRobot::AutoCubeSide(){
	int Speed_Auto_Turn;
	// assigns direction for turning based on location
	if(Starting_Position==1){
			Speed_Auto_Turn		=	Speed_Drive_Auto;
	}
	else{
			Speed_Auto_Turn		=	-Speed_Drive_Auto;
	}
	//AUTO
	if(loopCount<Time_Foward_Side1){//insert time
		pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
	}
	else{
	if(loopCount<Time_Turn_Side){//turn 90 deg right
			pDriveTrain->TankDrive(Speed_Auto_Turn,-Speed_Auto_Turn);
		}
		else{
		if(loopCount<Time_Foward_Side2){
			pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
		}
		else{
		if(loopCount<Time_Elevator){
			pElevator->Up1(Speed_Elevator_Auto);
		}
		else{
			pElevator->Stop();
			Stage1Stop();
			AirOut();
			pIntake->Out();
			}
			}
		 }
	}
	return;
}

void PowerUpRobot::AutoCubeCenter(){
	if(loopCount<Time_Elevator){
				Stage1Start();
				pElevator->Up1(Speed_Elevator_Auto);
			}
			else{
				pElevator->Stop();
				Stage1Stop();
				AirOut();
				pIntake->Out();
				}
	return;
}

void PowerUpRobot::RunClimber()
{
	if ( RightTrigger > .2 )
	{
		pClimber->Climb();
	}
	else
	{
		if ( LeftTrigger > .2 )
		{
			pClimber->Lower();
		}
		else
		{
			pClimber->StopClimber();
		}
	}

	return;
}

void PowerUpRobot::RunIntake(){
	// Robot running? great!- Mari
	//thank you so much for your insightful and useful comments
	if(pIntakeButton->Get()){
		pIntake->In();
	}
	else{
		if(pOuttakeButton->Get()){
			pIntake->Out();
		}
		else{
			pIntake->Stop();
		}
	}
	return;
}


/*
void PowerUpRobot::SolenoidState(bool state){
	if(state){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	}
	else{
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	}
}
*/


void PowerUpRobot::AirOut(){
	//Solenoid1.Set(frc::DoubleSolenoid::Value::kOff);
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::AirIn(){
	//Solenoid1.Set(frc::DoubleSolenoid::Value::kOff);
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}


void PowerUpRobot::SolenoidUpdate(){
	if(pAirOut->Get()){
			//SolenoidState(false);
		AirOut();
	}
	if(pAirIn->Get()){
			//SolenoidState(true);
		AirIn();
	}
	return;
}

/*
void   PowerUpRobot::RunElevator(){
	//need to add pneumatics stuff
	// Stage 1:

	if(pElv1Up->Get()){
			    pElevator->Up1();
		}
	else{
		if(pElv1Down->Get()){
			pElevator->Up1();
		}

		else{
				pElevator->Stop();
				Stage1Stop();
		}
	}
	//Stage 2:
	if(pElv2Up->Get()){
					pElevator->Up2();
			}
	else{
		if(pElv2Down->Get()){
			pElevator->Up2();
		}
		else{
				pElevator->Stop2();
				Stage2Stop();
		}
		}
	StageUpdate();
return;
}
*/
void   PowerUpRobot::RunElevator(){
	//need to add pneumatics stuff
	//done(?)
	// Stage 1:

//	if(pCCI1->GetY()!=0){
	if(LeftY > .3 || LeftY < -.3){ // needs a tolerance or it'll jiggle around all the time
			    pElevator->Up1(-LeftY);
		}
	else{
				pElevator->Stop();
				Stage1Stop();
	}
	//Stage 2:
//	if(pCCI1->GetY()!=0){
	if(RightY > .3 || RightY < -.3){
				pElevator->Up2(-RightY);
			}
	else{
				pElevator->Stop2();
				Stage2Stop();
		}
	StageUpdate();
return;
}

void PowerUpRobot::Stage1Stop(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::Stage1Start(){
	Solenoid2.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}

void PowerUpRobot::Stage2Stop(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::Stage2Start(){
	Solenoid3.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}

void PowerUpRobot::StageUpdate(){
			if(LeftY > .3 || LeftY < -.3){
				Stage2Start();
			}
			if(RightY > .3 || RightY < -.3){
				Stage1Start();
			}
}

void PowerUpRobot::Delay(){
	if(DelayCounter<Time_Elevator_Delay){
		DelayCounter=0;
		Delayed = true;
	}
	else{
		DelayCounter=0;
		Delayed = false;
	}
}

bool PowerUpRobot::FMSFetch(){
	//confirms valid FMS values before starting auto
	if((gameData=="L"||gameData=="R")&&(Starting_Position==1||Starting_Position==2||Starting_Position==3)){
		return true;
	}
	else{
		return false;
	}
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::CalcAutoModeTimings()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
// Obtains the autonomous mode switch values from the robot and determines
// which autonomous mode we want to run.
//------------------------------------------------------------------------------

void PowerUpRobot::CalcAutoModeTimings()
{
    //put all the auto timing addition stuff here

	return;
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::GetAutoModeSwitches()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
// Obtains the autonomous mode switch values from the robot and determines
// which autonomous mode we want to run.
//------------------------------------------------------------------------------

void PowerUpRobot::GetAutoModeSwitches()
{
	//all the auto mode possibilities
	if ( pAutoSwitch1->Get() )
	{
		if ( pAutoSwitch2->Get() )
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 8 - On On On
				//autoMode         = kAM8LowBarShootHigh2;
				//autoModeSelected = 8;
			}
			else
			{
				// Autonomous Mode 6 - On On Off
				//autoMode = kAM6LowBarShootHigh;
				//autoModeSelected = 6;
			}
		}
		else
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 7 - On Off On
				//autoMode = kAM7LowBarShootLow2;
				//autoModeSelected  = 7;
			}
			else
			{
				// Autonomous Mode 5 - On Off Off
				//autoMode = kAM5LowBarShootLow;
				//autoModeSelected  = 5;
			}
		}
	}
	else
	{
		if ( pAutoSwitch2->Get() )
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 4 - Off On On
				//autoMode = kAM4TurnLowBar;
				//autoModeSelected = 4;
			}
			else
			{
				// Autonomous Mode 2 - Off On Off
				//autoMode = kAM2DriveLowBar;
				//autoModeSelected  = 2;
			}
		}
		else
		{
			if ( pAutoSwitch3->Get() )
			{
				// Autonomous Mode 3 - Off Off On
				//autoMode = kAM3DriveOnly;
				//autoModeSelected = 3;
			}
			else
			{
				// Autonomous Mode 1 - Off Off Off
				//autoMode = kAM1Off;
				//autoModeSelected = 1;
			}
		}
	}
	return;
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::RunAutonomousMode()
// Type:	Public accessor for PowerUpRobot class
//------------------------------------------------------------------------------
// Executes autonomous mode functions.
//------------------------------------------------------------------------------
void PowerUpRobot::RunAutonomousMode()
{
//	AMDriveFwd();
	return;
}
//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::ShowAMStatus()
// Type:	Public accessor for PowerUpRobot class
//------------------------------------------------------------------------------
// Displays information about autonomous mode for debugging purposes.
//------------------------------------------------------------------------------
void PowerUpRobot::ShowAMStatus()
{
//	SmartDashboard::PutNumber("Autonomous Mode",autoModeSelected);
	SmartDashboard::PutNumber("Loop Counter",loopCount);
	SmartDashboard::PutBoolean("R AM Switch 1",pAutoSwitch1->Get());
	SmartDashboard::PutBoolean("R AM Switch 2",pAutoSwitch2->Get());
	SmartDashboard::PutBoolean("R AM Switch 3",pAutoSwitch3->Get());

	return;
}

//Auto Mode 1: do nothing
void PowerUpRobot::AM1Off()
{
	// Do nothing
	// Yay we did nothing
	//good job everyone
	return;
}

//------------------------------------------------------------------------------
// METHOD:  PowerUpRobot::AMDriveFwd()
// Type:	Public accessor for SteamWorksRobot class
//------------------------------------------------------------------------------
// Drives forward
//------------------------------------------------------------------------------
void PowerUpRobot::AMDriveFwd()
{
//	if(loopCount < 200)
//		pDriveTrain->TankDrive(AM_DRIVE_FWD_RIGHT_FAST_SPEED,AM_DRIVE_FWD_LEFT_FAST_SPEED);
//	else
//		pDriveTrain->TankDrive(AM_DRIVE_STOP,AM_DRIVE_STOP);
	return;
}

