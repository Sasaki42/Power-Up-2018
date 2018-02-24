//------------------------------------------------------------------------------
// TEAM 1280 - SAN RAMON VALLEY HIGH SCHOOL RAGIN' C-BISCUITS
// 2018 POWER UP ROBOT CODE

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

	public:

		PowerUpRobot();
		~PowerUpRobot();

		void   RobotInit();
		void   DisabledInit();
		void   AutonomousInit();
		void   TeleopInit();
		void   DisabledPeriodic();
		void   AutonomousPeriodic();
		void   TeleopPeriodic();

	private:

		static const uint JS_PORT_LEFT           =  0;
		static const uint JS_PORT_RIGHT			 =  1;
		static const uint CCI_PORT1        	     =  2;

		// Joystick Button (right joystick)
		static const uint CAMERA_LIGHT_SW_CH     =  1;

		// Driver Station CCI1 Channels (Uses joystick button references)
		static const uint Intake_SW_CH			 =  6;
		static const uint Outtake_SW_CH			 =  5;
		static const uint AIROUT_SW_CH 			 =  1;
		static const uint AIRIN_SW_CH			 =  2;

		// roboRio GPIO Channels
		static const uint AUTONOMOUS_SW_1_CH        =  0;
		static const uint AUTONOMOUS_SW_2_CH        =  1;
		static const uint AUTONOMOUS_SW_3_CH        =  2;

		// roboRio PWM Channels (PWM = Pulse width modulation)

		static const uint RIGHT_FRONT_MOTOR_CH	  =  9;
		static const uint RIGHT_REAR_MOTOR_CH     =  8;
		static const uint LEFT_FRONT_MOTOR_CH	  =  3;
		static const uint LEFT_REAR_MOTOR_CH	  =  2;
		static const uint LEFT_INTAKE_MOTOR_CH	  =  0;
		static const uint RIGHT_INTAKE_MOTOR_CH	  =  1;
		static const uint CLIMBER_MOTOR1_CH		  =  6;
		static const uint CLIMBER_MOTOR2_CH		  =  7;
		static const uint ELEVATOR_MOTOR1_CH 	  =  4;
		static const uint ELEVATOR_MOTOR2_CH	  =  5;

		// roboRio Solenoid Channels
		static const uint Compressor_CH 		  =  0;
		static const uint Solenoid1a_CH			  =  0;
		static const uint Solenoid2a_CH	   		  =  1;
		static const uint Solenoid1b_CH			  =  2;
		static const uint Solenoid2b_CH	   		  =  3;
		static const uint Solenoid1c_CH			  =  4;
		static const uint Solenoid2c_CH	   		  =  5;

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

	enum autoModeStates {kAM1Of};
	std::string gameData;
	int Starting_Position;

	Joystick		 *pDriveStickLeft;
	Joystick		 *pDriveStickRight;
	Joystick         *pCCI1;

	JoystickButton   *pCameraLightButton;
    JoystickButton 	 *pIntakeButton;
    JoystickButton   *pOuttakeButton;
    JoystickButton   *pAirOut;
    JoystickButton   *pAirIn;

    DigitalInput    *pAutoSwitch1;
    DigitalInput    *pAutoSwitch2;
    DigitalInput    *pAutoSwitch3;

    //the points for all the mechanisms
	RobotDrive		*pDriveTrain;
	Climber			*pClimber;
	Intake 			*pIntake;
	Elevator 		*pElevator;
	Compressor 		*pCompressor;

	frc::DoubleSolenoid Solenoid1  {Solenoid1a_CH, Solenoid2a_CH};
	frc::DoubleSolenoid Solenoid2 {Solenoid1b_CH, Solenoid2b_CH};
	frc::DoubleSolenoid Solenoid3 {Solenoid1c_CH, Solenoid2c_CH};

	// Class variables to track look (packet) counts
	uint   loopCount;
	uint   DelayCounter;

	// Driver station and robot input gathering methods
	void   GetDriverStationInput();
	void   ShowDSValues();
	void   GetRobotSensorInput();
	void   ShowRobotValues();

	// Autonomous mode methods
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

	//Drive Train
	float rightDriveSpeed;
	float leftDriveSpeed;

	float LeftY;
	float RightY;
	float LeftTrigger;
	float RightTrigger;

	const double period =0.0;
};

START_ROBOT_CLASS(PowerUpRobot)

PowerUpRobot::PowerUpRobot()
{
	// Define joysticks & CCI
	pDriveStickLeft		   = new Joystick(JS_PORT_LEFT);
	pDriveStickRight	   = new Joystick(JS_PORT_RIGHT);
	pCCI1                  = new Joystick(CCI_PORT1); // CCI uses joystick object

	pCameraLightButton     = new JoystickButton(pDriveStickRight,CAMERA_LIGHT_SW_CH);

	pIntakeButton 		   = new JoystickButton(pCCI1,Intake_SW_CH);
	pOuttakeButton 		   = new JoystickButton(pCCI1,Outtake_SW_CH);
	pAirOut				   = new JoystickButton(pCCI1,AIRIN_SW_CH);
	pAirIn				   = new JoystickButton(pCCI1,AIROUT_SW_CH);

	pCompressor 		   = new Compressor(Compressor_CH);

	pAutoSwitch1         = new DigitalInput(AUTONOMOUS_SW_1_CH);
	pAutoSwitch2         = new DigitalInput(AUTONOMOUS_SW_2_CH);
	pAutoSwitch3         = new DigitalInput(AUTONOMOUS_SW_3_CH);

	pDriveTrain		     = new RobotDrive(LEFT_FRONT_MOTOR_CH,LEFT_REAR_MOTOR_CH,
							RIGHT_FRONT_MOTOR_CH, RIGHT_REAR_MOTOR_CH);
	pClimber		     = new Climber(CLIMBER_MOTOR1_CH, CLIMBER_MOTOR2_CH);
	pIntake 			 = new Intake(LEFT_INTAKE_MOTOR_CH, RIGHT_INTAKE_MOTOR_CH);
	pElevator 			 = new Elevator(ELEVATOR_MOTOR1_CH, ELEVATOR_MOTOR2_CH);

	loopCount      		= 0;
	DelayCounter		= 0;

	rightDriveSpeed       = 0.0;
	leftDriveSpeed        = 0.0;

	LeftY				  = 0.0;
	RightY				  = 0.0;
	LeftTrigger  		  = 0.0;
	RightTrigger		  = 0.0;

	Starting_Position     = 1;
	return;
}

PowerUpRobot::~PowerUpRobot()
{
}

void PowerUpRobot::RobotInit()
{
#ifdef CONSOLE
	SmartDashboard::init();
#endif

}

void PowerUpRobot::DisabledInit()
{
	loopCount  = 0;
	AirOut();
	return;
}

void PowerUpRobot::AutonomousInit()
{
	loopCount  = 0;
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	GetRobotSensorInput();
	Starting_Position = frc::DriverStation::GetInstance().GetLocation();
	gameData = "L";

	return;
}

void PowerUpRobot::TeleopInit()
{
	loopCount      = 0;
	DelayCounter   = 0;

	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	pCompressor->SetClosedLoopControl(true);
	return;
}

void PowerUpRobot::DisabledPeriodic()
{
	loopCount++;
	AirOut();
	GetRobotSensorInput();

#ifdef CONSOLE
	ShowRobotValues();
#endif
	return;
}

void PowerUpRobot::AutonomousPeriodic()
{
	if(FMSFetch()){
	loopCount++;
	DelayCounter++;
	GetRobotSensorInput();
	AutoSwitch();
	SmartDashboard::PutNumber("LoopCount",loopCount);
	}
	else{
		// get the starting Starting_Position
		Starting_Position = frc::DriverStation::GetInstance().GetLocation();
		 // get sides of switches, scale
		gameData= frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}
	return;
}

void PowerUpRobot::TeleopPeriodic()
{
	GetDriverStationInput();
	GetRobotSensorInput();

	loopCount++;
	DelayCounter++;

    RunClimber();
    RunIntake();
    RunElevator();
    SolenoidUpdate();
	pDriveTrain->TankDrive(-leftDriveSpeed,-rightDriveSpeed);
	return;
}

void PowerUpRobot::GetDriverStationInput()
{
	rightDriveSpeed		=  pDriveStickRight->GetY();
	leftDriveSpeed		=  pDriveStickLeft->GetY();

	LeftY               = pCCI1->GetRawAxis(1);
	RightY              = pCCI1->GetRawAxis(5);
	LeftTrigger         = pCCI1->GetRawAxis(2);
	RightTrigger        = pCCI1->GetRawAxis(3);

#ifdef CONSOLE
    ShowDSValues();
#endif
	return;
}
#ifdef CONSOLE

void PowerUpRobot::ShowDSValues()
{
	// Show the values for driver station inputs
	SmartDashboard::PutNumber ("Joystick Right",pDriveStickRight->GetY());
	SmartDashboard::PutNumber ("Joystick Left",pDriveStickLeft->GetY());
	SmartDashboard::PutBoolean("DS Intake Robot",pIntakeButton->Get());
	SmartDashboard::PutBoolean("DS Air Out    ",pAirOut->Get());
	SmartDashboard::PutBoolean("DS Air In     ",pAirIn->Get());
	return;
}
#endif

void PowerUpRobot::GetRobotSensorInput()
{
#ifdef CONSOLE
	ShowRobotValues();
#endif

	return;
}
#ifdef CONSOLE

void PowerUpRobot::ShowRobotValues()
{
	SmartDashboard::PutBoolean("R AM Switch 1",pAutoSwitch1->Get());
	SmartDashboard::PutBoolean("R AM Switch 2",pAutoSwitch2->Get());
	SmartDashboard::PutBoolean("R AM Switch 3",pAutoSwitch3->Get());
	return;
}
#endif

void PowerUpRobot::AutoSwitch(){
	switch(Starting_Position){
								case 1: 		//LEFT Side
								if(gameData[0]=='L'){
									//put cube on switch
									AutoCubeSide();
								}
								else if(gameData[0]=='R'){
								// go to null
									if (loopCount < Time_Foward_ToNull) {
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else{

									}
								}
								else{

								}
								break;
							case 2: 			//CENTER field
								if(gameData[0]=='L'){
									// go left
									if (loopCount < Time_Turn_Center) {
									pDriveTrain->TankDrive(-Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else if(loopCount < Time_Foward_Center){
									}
									else{
									AutoCubeCenter();
									}
									}
								else if(gameData[0]=='R'){
									// go right
									if (loopCount < Time_Turn_Center) {
									pDriveTrain->TankDrive(Speed_Drive_Auto,-Speed_Drive_Auto);
									}
									else if(loopCount < Time_Foward_Center){
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else{
									AutoCubeCenter();
									}
								}
								else{

								}
								break;
							case 3: 	     	//RIGHT Side
								if(gameData[0]=='L'){
									//go to null
									if (loopCount < Time_Foward_ToNull) {//insert time
									pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
									}
									else{
									}
									}
									else if(gameData[0]=='R'){
									//put cube on switch
										AutoCubeSide();
									}
									else{

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

	if(loopCount<Time_Foward_Side1){
		pDriveTrain->TankDrive(Speed_Drive_Auto,Speed_Drive_Auto);
	}
	else{
	if(loopCount<Time_Turn_Side){
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

void PowerUpRobot::AirOut(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kForward);
	return;
}

void PowerUpRobot::AirIn(){
	Solenoid1.Set(frc::DoubleSolenoid::Value::kReverse);
	return;
}


void PowerUpRobot::SolenoidUpdate(){
	if(pAirOut->Get()){
		AirOut();
	}
	if(pAirIn->Get()){
		AirIn();
	}
	return;
}

void   PowerUpRobot::RunElevator(){
	//tolerance for joystick
	if(LeftY > .3 || LeftY < -.3){
			    pElevator->Up1(-LeftY);
		}
	else{
				pElevator->Stop();
				Stage1Stop();
	}
	//Stage 2:
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
	if((gameData=="L"||gameData=="R")&&(Starting_Position==1||Starting_Position==2||Starting_Position==3)){
		return true;
	}
	else{
		return false;
	}
}

