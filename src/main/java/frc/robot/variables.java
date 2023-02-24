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

    // button channels 
    int buttonAChannel = 1; 
    int buttonBChannel = 2; 
    int buttonYChannel = 4;
    int buttonRBChannel = 5; 
    int buttonLBChannel = 6; 
    int buttonLeftChannel = 9;
    int buttonRightChannel = 10;

    // safty stuff 
    double safetyFactor = 0.5; 
    double turnRate = 0.5; 

    // pid stuff 
    double distanceKP = 0.05;
    double gyroKP = 0.025;

    // piston timings 
    double pistonQuick = 50; 
    double pistonSlow = 100; 

    // lower jaws pid values 
    double lower_jaw_kp = 0.2; 
    double lower_jaw_ki = 0.01; 
    double lower_jaw_kd = 0; 
 
}