package frc.robot;

public class variables {
    int leftFrontMotorID = 2; 
    int leftBackMotorID = 1;
    int rightFrontMotorID = 4;
    int rightBackMotorID = 3;  
    int jawLowerMotorID = 5;  
    int jawUpperMotorID = 6; 
    int lowerIntakeMortorID = 7;

    // joy stick axises
    int joyStickLeft_AxisY = 1; 
    int joyStickRight_AxisX = 4;
    int leftTriggerButton = 2; 
    int rightTriggerButton = 3; 

    // button channels 
    int buttonAChannel = 1; 
    int buttonBChannel = 2; 
    int buttonXChannel = 3; 
    int buttonYChannel = 4;
    int buttonRBChannel = 5; 
    int buttonLBChannel = 6; 
    int leftJoyStickButton = 9;
    int rightJoyStickButton = 10;
    

    // safty stuff 
    double safetyFactor = 0.5; 
    double turnRate = 0.15; 

    // pid stuff 
    double distanceKP = 0.05;
    double gyroKP = 0.025;

    
    // piston timings 
    double pistonQuick = 50; 
    double pistonSlow = 300; 

    // lower jaws pid values 
    double lower_jaw_kp = 0.7; 
    double lower_jaw_ki = 0.05; 
    double lower_jaw_kd = 0; 

    double upper_jaw_kp = 0.0375; 
    double upper_jaw_ki = 0.005; 
    double upper_jaw_kd = 0; 

    // mechanism home position 
    double homePositionUpperJaw = 6; 
    double homePositionLowerJaw = 0; 

    // clamp durring shoot 
    double clampShootPositionUpperJaw = -5; 

    // mechanism shoot position 
    double shootPositionUpperJaw = 0; 
    double shootPositionLowerJaw = -17; 

    // mechanism lower position 
    double pickUpPositionUpperJaw = 5; 
    double pickUpPositionLowerJaw = -28; 

    // cone lower position 
    double pickUpPositionUpperJawConeClamp = 0; 


    // intake speed 
    double intakeLowerSpeed = 0.5; 
    double intakeUpperSpeed = 0.5; 
    double outtakeUpperSpeed = 0.7;
    double autoOuttakeUpperSpeed = 0.30;
    double winchSpeed = 0.85; 

    // double gear solonoid variables 
    int gearSolonoidPortOne = 2;
    int gearSolonoidPortTwo = 3; 

    // double piston soloind variable 
    int pistonSolonoidPortOne = 0; 
    int pistonSolonoidPortTwo = 1; 


    // double pwm port stuff 
    int lowerIntakePort = 0; 
    int upperRightIntakePort = 1; 
    int upperLeftIntakePort = 2; 
    int winchPort = 4; 
    

    // digital input 
    int limitSwitchBackPort = 0; 
    int limitSwitcFrontPort = 1; 
    int intakeBumperSwitchPort = 2; 

    // autonomous contants 
    double rotationsPerFoot = 2.26; 

    // drive pid constants 
    double drive_kp = 0.01;  
    double drive_ki = 0; 
    double drive_kd = 0; 

    // gryo drive PID Constants 
    double gryo_kp = 0.005; 
    double gryo_ki = 0; 
    double gryo_kd = 0; 

    // double pitch PID constants 
    double pitch_kp = 0.05; 
    double pitch_ki = 0; 
    double pitch_kd = 0; 

    // double velocity PID constants
    double velocity_kp = 0.0003; 
    double velocity_ki = 0.0001; 
    double velocity_kd = 0; 

    // autonomous speed
    double autonomousSpeedFast = 400;
    double distanceToClearCommunityOnTheSide = -30;
}