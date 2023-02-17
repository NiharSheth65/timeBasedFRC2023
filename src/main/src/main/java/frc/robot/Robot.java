// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

// public class Robot extends IterativeRobot {

//     public void robotInit() {
//       CameraServer.getInstance().startAutomaticCapture();
//     }
//   }
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  variables var = new variables();

  private Joystick Joy = new Joystick(0);

  // declaring motor ids, still need to check if it is correct

  // creating motor objects
  private CANSparkMax leftMotorFront;
  private CANSparkMax leftMotorBack;
  private CANSparkMax rightMotorFront;
  private CANSparkMax rightMotorBack;
  private CANSparkMax jawLowerMotor; 

  // creating enencoder
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder jawLowerEncoder; 

  // creating compressor object
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // creating double solonoids
  DoubleSolenoid gearSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid pistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  // intial setpoints
  private double LeftMotor_Setpoint = 0;
  private double RightMotor_Setpoint = 0;
  private double Jaw_Lower_Setpoint = 0; 
  private double Jaw_Lower_Speed; 

  private double rightEncoderPosition;
  private double leftEncoderPosition;
  private double jawLowerEncoderPosition; 

  // actually find the gear ratio and replace, 1 is a place holder
  private double ticksPerFoot = 1;

  // compressor logic
  boolean enabled = pcmCompressor.isEnabled();

  // nav x stuff
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  double navxAutonomousPosition;

  @Override
  public void robotInit() {
    leftMotorFront = new CANSparkMax(var.leftFrontMotorID, MotorType.kBrushless);
    leftMotorBack = new CANSparkMax(var.leftBackMotorID, MotorType.kBrushless);
    rightMotorFront = new CANSparkMax(var.rightFrontMotorID, MotorType.kBrushless);
    rightMotorBack = new CANSparkMax(var.rightBackMotorID, MotorType.kBrushless);
    jawLowerMotor = new CANSparkMax(var.jawLowerMotorID, MotorType.kBrushless); 

    // CameraServer.startAutomaticCapture();
    // final UsbCamera camera;

    // camera = CameraServer.startAutomaticCapture();
    // camera.getVideoMode();
    // camera.setResolution(250, 250);

    leftMotorFront.restoreFactoryDefaults();
    leftMotorBack.restoreFactoryDefaults();
    rightMotorFront.restoreFactoryDefaults();
    rightMotorBack.restoreFactoryDefaults();
    jawLowerMotor.restoreFactoryDefaults(); 
    // the back motors will be following the front motors

    leftMotorFront.setInverted(true);
    leftMotorBack.setInverted(true);

    leftMotorBack.follow(leftMotorFront);
    rightMotorBack.follow(rightMotorFront);

    rightEncoder = rightMotorFront.getEncoder();
    leftEncoder = leftMotorFront.getEncoder();
    jawLowerEncoder = jawLowerMotor.getEncoder(); 
  
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    leftMotorFront.getEncoder().setPosition(0);
    rightMotorFront.getEncoder().setPosition(0);
    navx.reset();
    navx.zeroYaw();
    navxAutonomousPosition = navx.getYaw();
  }

  @Override
  public void autonomousPeriodic() {

    rightEncoderPosition = (leftEncoder.getPosition() * ticksPerFoot);
    leftEncoderPosition = (rightEncoder.getPosition() * ticksPerFoot);

    double distance = (leftEncoderPosition + rightEncoderPosition) / 2;

    double gyroDegreesTurned = navx.getYaw() - navxAutonomousPosition;

    // SmartDashboard.putNumber("gryo Angle", navx.getYaw()); 
    
    double angleTotal = 0; 
    double timesRan = 0; 

    if(navx.getYaw() >= -0.5 && navx.getYaw() <= 0.5){
      rightMotorFront.set(0.125);
      leftMotorFront.set(0.125);
    }

    else if(navx.getYaw() > 0.5){
      rightMotorFront.set(0.22); 
      leftMotorFront.set(0.125); 
    }
    
    else if(navx.getYaw() < -0.5){
      rightMotorFront.set(0.125);
      leftMotorFront.set(0.175);
    }

    else{
      rightMotorFront.set(0);
      leftMotorFront.set(0);
    }

    timesRan++;
    angleTotal += navx.getYaw(); 

    double medianAngle = angleTotal/timesRan; 
    // SmartDashboard.putNumber("median number", medianAngle); 

  }

  @Override
  public void teleopInit() {
    navx.reset();
    navx.zeroYaw();
    jawLowerMotor.getEncoder().setPosition(0);
  }

  @Override
  public void teleopPeriodic() {

    double gyroYaw = navx.getYaw();
    double gyroRoll = navx.getRoll();
    double gyroPitch = navx.getPitch();
    double gyroAngle = navx.getAngle();
    double gyroDisplacemntX = navx.getDisplacementX();
    double gyroDisplacemntY = navx.getDisplacementY();
    double gyroDisplacemntZ = navx.getDisplacementZ();

    // System.out.println("gryo pitch: " + gyroPitch);
    // SmartDashboard.putNumber("gyro yaw", gyroYaw);
    // SmartDashboard.putNumber("gyro roll", gyroRoll);
    // SmartDashboard.putNumber("gyro pitch", gyroPitch);
    // SmartDashboard.putNumber("gyro angle", gyroAngle);
    // SmartDashboard.putNumber("gyro displacemnt x", gyroDisplacemntX);
    // SmartDashboard.putNumber("gyro displacement y", gyroDisplacemntY);
    // SmartDashboard.putNumber("gyro displacement z", gyroDisplacemntZ);

    double safetyFactor = 0.5;
    double joyStick_left_Y = -Joy.getRawAxis(var.joyStickLeft_AxisY) * safetyFactor;
    double joyStick_right_X = -Joy.getRawAxis(var.joyStickRight_AxisX) * safetyFactor;

    // Acceleration Control _ Needs a time factor included

    double turnRate = 0.25;
    double LeftMotor_TargetPoint = joyStick_left_Y - (joyStick_right_X * turnRate);
    double RightMotor_TargetPoint = joyStick_left_Y + (joyStick_right_X * turnRate);
    LeftMotor_Setpoint = Acceleration_contol(LeftMotor_TargetPoint, LeftMotor_Setpoint);
    RightMotor_Setpoint = Acceleration_contol(RightMotor_TargetPoint, RightMotor_Setpoint);

    rightMotorFront.set(RightMotor_Setpoint);
    leftMotorFront.set(LeftMotor_Setpoint);

    boolean buttonA = Joy.getRawButton(var.buttonAChannel);
    boolean buttonB = Joy.getRawButton(var.buttonBChannel);

    boolean buttonRB = Joy.getRawButton(var.buttonRBChannel);
    boolean buttonLB = Joy.getRawButton(var.buttonLBChannel);

    boolean buttonRight = Joy.getRawButton(var.buttonRightChannel); 
    boolean buttonLeft = Joy.getRawButton(var.buttonLeftChannel); 

    if (buttonRB) {
      gearSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

    else if (buttonLB) {
      gearSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    if (buttonRight) {
      pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  
    else if (buttonLeft) {
       pistonSolenoid.set(DoubleSolenoid.Value.kForward);
    }


    // --------------------------------------------------------------------------------------------------------------------- //
    // jaw stuff 

    jawLowerEncoderPosition = (jawLowerEncoder.getPosition()); 
    double jawAngle = jawLowerEncoderPosition * 7.5; 
    SmartDashboard.putNumber("Jaw Angle", jawAngle);


    if (buttonA) {
        // Here, we run P control like normal, with a constant setpoint of 30in.
        // Jaw_Lower_Measured = (jawLowerEncoder.getPosition() * 12)/90; 
        Jaw_Lower_Setpoint = 90; 
        // jawLowerMotor.set(pidJaw(Jaw_Lower_Setpoint, jawAngle));
        pidJaw(Jaw_Lower_Setpoint, jawAngle); 

    }

    else if(buttonB){
        Jaw_Lower_Setpoint = 0; 
        // jawLowerMotor.set(pidJaw(Jaw_Lower_Setpoint, jawAngle));
        pidJaw(Jaw_Lower_Setpoint, jawAngle); 
    } 

    else {
        // Otherwise, set to 0
        jawLowerMotor.set(0);
    }
    

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }


  public double Acceleration_contol(double TargetPoint, double Setpoint) {

    double agressionFactor = 5;

    // return (Setpoint + (TargetPoint - Setpoint)/agressionFactor);

    if (((TargetPoint > 0) && (Setpoint < 0) || (TargetPoint < 0) && (Setpoint > 0))) {

      agressionFactor = 10;
    }

    return (Setpoint + (TargetPoint - Setpoint) / agressionFactor);

  }

  public void kickTheRobot() {
    if (navx.getYaw() < 1 && navx.getYaw() > -1) {
      rightMotorFront.set(0);
      leftMotorFront.set(0);
    }

    else if (navx.getYaw() > 1) {
      rightMotorFront.set(0.05);
      leftMotorFront.set(-0.05);
    }

    // turn left faster
    else if (navx.getYaw() < -1) {
      rightMotorFront.set(-0.05);
      leftMotorFront.set(0.05);
    }

    rightMotorFront.set(0.25);
    leftMotorFront.set(0.25);

    // SmartDashboard.putNumber("gryo value", navx.getYaw());

  }

  public void pidJaw(double setPoint, double measurment){
    double error = setPoint - measurment; 
    double kp = 0.01; 
 
    // SmartDashboard.putNumber("speed", speed); 

    while(error != 0 ){
        double speed = error * kp; 
        double newMeasurment = (jawLowerEncoder.getPosition() * 7.5); 
        error = setPoint - newMeasurment; 
        SmartDashboard.putNumber("error", error); 
        jawLowerMotor.set(speed);

        if(error < 1 && error > -1){
            break; 
        } 
    }



    
}
    
    
    
    
    // return speed; 

  // work for tommorow, start converting encoder values into degrees
  // use that as a measured value 
  // start determing set point values 
  // try turning to those values 
}
