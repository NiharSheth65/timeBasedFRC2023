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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;

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
  private CANSparkMax jawUpperMotor;
  private VictorSP victor = new VictorSP(0);
  
  // creating enencoder
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder jawLowerEncoder;
  private RelativeEncoder jawUpperEncoder;

  // creating compressor object
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // creating double solonoids
  DoubleSolenoid gearSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, var.gearSolonoidPortOne, var.gearSolonoidPortTwo);
  DoubleSolenoid pistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, var.pistonSolonoidPortOne, var.pistonSolonoidPortTwo);

  // intial setpoints
  private double LeftMotor_Setpoint = 0;
  private double RightMotor_Setpoint = 0;
  private double Jaw_Lower_Setpoint = 0;
  private double Jaw_Upper_Setpoint = 0;

  private double rightEncoderPosition;
  private double leftEncoderPosition;
  private double jawLowerEncoderPosition;
  private double jawUpperEncoderPosition;

  // compressor logic
  boolean enabled = pcmCompressor.isEnabled();

  // nav x stuff
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  double navxAutonomousPosition;

  // auto chooser stuff
  private static final String kMiddleAuto = "middle";
  private static final String kRightAuto = "right";
  private static final String kLeftAuto = "left";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // pid controllers 
  PIDController jawLowerMotorPIDControllers = new PIDController(var.lower_jaw_kp, var.lower_jaw_ki, var.lower_jaw_kd);
  PIDController jawUpperMotorPIDControllers = new PIDController(var.upper_jaw_kp, var.lower_jaw_ki, var.lower_jaw_kd); 
  
  // piston variables
  boolean pistonLaunched; 
  boolean jawLowerInPosition; 
  boolean jawUpperInPosition; 

  @Override
  public void robotInit() {

    // create all variable objects
    leftMotorFront = new CANSparkMax(var.leftFrontMotorID, MotorType.kBrushless);
    leftMotorBack = new CANSparkMax(var.leftBackMotorID, MotorType.kBrushless);
    rightMotorFront = new CANSparkMax(var.rightFrontMotorID, MotorType.kBrushless);
    rightMotorBack = new CANSparkMax(var.rightBackMotorID, MotorType.kBrushless);
    jawLowerMotor = new CANSparkMax(var.jawLowerMotorID, MotorType.kBrushless);
    jawUpperMotor = new CANSparkMax(var.jawUpperMotorID, MotorType.kBrushless);

    // reset all motors
    leftMotorFront.restoreFactoryDefaults();
    leftMotorBack.restoreFactoryDefaults();
    rightMotorFront.restoreFactoryDefaults();
    rightMotorBack.restoreFactoryDefaults();
    jawLowerMotor.restoreFactoryDefaults();

    // the back motors will be following the front motors
    leftMotorFront.setInverted(true);
    leftMotorBack.setInverted(true);

    // get back motors to follow front motors
    leftMotorBack.follow(leftMotorFront);
    rightMotorBack.follow(rightMotorFront);

    // setting the encoders
    rightEncoder = rightMotorFront.getEncoder();
    leftEncoder = leftMotorFront.getEncoder();
    jawLowerEncoder = jawLowerMotor.getEncoder();
    jawUpperEncoder = jawUpperMotor.getEncoder();

    // start camera
    CameraServer.startAutomaticCapture();

    // auto chooser
    m_chooser.setDefaultOption("Middle Auto", kMiddleAuto);
    m_chooser.addOption("Right Auto", kRightAuto);
    m_chooser.addOption("Left Auto", kLeftAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    jawLowerMotor.getEncoder().setPosition(0);
    jawUpperMotor.getEncoder().setPosition(0);

    jawLowerMotor.set(jawLowerMotorPIDControllers.calculate(jawLowerEncoder.getPosition(), 0));
    jawUpperMotor.set(jawUpperMotorPIDControllers.calculate(jawUpperEncoder.getPosition(), 0));    

    pistonLaunched = false; 
    jawLowerInPosition = true; 
    jawUpperInPosition = true; 
    
    offPiston();
    jawLowerMotorPIDControllers.reset();
    jawUpperMotorPIDControllers.reset();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {

    // set encoders to 0
    leftMotorFront.getEncoder().setPosition(0);
    rightMotorFront.getEncoder().setPosition(0);

    // reset and zero the navx
    navx.reset();
    navx.zeroYaw();

    // set automous position to 0
    navxAutonomousPosition = navx.getYaw();

    // option chooser
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    // get encoder readings and convert them to desired units
    rightEncoderPosition = (leftEncoder.getPosition() / var.ticksPerFoot);
    leftEncoderPosition = (rightEncoder.getPosition() / var.ticksPerFoot);

    // distance travalled
    double distanceTravelled = (rightEncoderPosition + leftEncoderPosition) / 2;

    double gyroSetpoint = navxAutonomousPosition;
    double navxYawReading = navx.getYaw();

    switch (m_autoSelected) {
      case kRightAuto:
        // Put custom auto code here
        break;
      case kLeftAuto:
        // put left code here
        break;
      case kMiddleAuto:
      default:
        // should clear doc and point line by this point
        double setDistance = 20;
        double distanceError = setDistance - distanceTravelled;
        
        if(distanceError > 0){
          goForward(setDistance, distanceTravelled, gyroSetpoint, navxYawReading, distanceError);
        }


        break;
    }

    SmartDashboard.putNumber("right ticks", rightEncoder.getPosition());
    SmartDashboard.putNumber("left ticks", leftEncoder.getPosition());

    SmartDashboard.putNumber("right voltage", leftMotorFront.getBusVoltage());
    SmartDashboard.putNumber("left voltage", leftMotorFront.getBusVoltage());

  }

  @Override
  public void teleopInit() {
    navx.reset();
    navx.zeroYaw();
    // jawLowerMotor.getEncoder().setPosition(0);
    // jawUpperMotor.getEncoder().setPosition(0);

    leftMotorFront.getEncoder().setPosition(0);
    rightMotorFront.getEncoder().setPosition(0);

    jawLowerMotor.getEncoder().setPosition(0);
    jawUpperMotor.getEncoder().setPosition(0);

    jawLowerMotor.set(jawLowerMotorPIDControllers.calculate(jawLowerEncoder.getPosition(), 0));
    jawUpperMotor.set(jawUpperMotorPIDControllers.calculate(jawUpperEncoder.getPosition(), 0));
    // pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
    
    pistonLaunched = false; 
    jawLowerInPosition = true; 
    jawUpperInPosition = true; 

    jawLowerMotorPIDControllers.reset();
    jawUpperMotorPIDControllers.reset();
    
    offPiston();
  }

  @Override
  public void teleopPeriodic() {

    rightEncoderPosition = rightEncoder.getPosition();
    leftEncoderPosition = leftEncoder.getPosition();
    SmartDashboard.putNumber("right encoder position teleOp", rightEncoderPosition);
    SmartDashboard.putNumber("left encoder position teleOp", leftEncoderPosition);

    // get joy stick readings
    double joyStick_left_Y = -Joy.getRawAxis(var.joyStickLeft_AxisY) * var.safetyFactor;
    double joyStick_right_X = -Joy.getRawAxis(var.joyStickRight_AxisX) * var.safetyFactor;

    // Acceleration Control _ Needs a time factor included
    double LeftMotor_TargetPoint = joyStick_left_Y - (joyStick_right_X * var.turnRate);
    double RightMotor_TargetPoint = joyStick_left_Y + (joyStick_right_X * var.turnRate);

    // calculate a speed for the motors
    LeftMotor_Setpoint = Acceleration_contol(LeftMotor_TargetPoint, LeftMotor_Setpoint);
    RightMotor_Setpoint = Acceleration_contol(RightMotor_TargetPoint, RightMotor_Setpoint);

    // set the motors to that speed
    rightMotorFront.set(RightMotor_Setpoint);
    leftMotorFront.set(LeftMotor_Setpoint);

    // read button a and b values
    boolean buttonA = Joy.getRawButton(var.buttonAChannel);
    boolean buttonB = Joy.getRawButton(var.buttonBChannel);
    boolean buttonY = Joy.getRawButton(var.buttonYChannel);
    boolean buttonX = Joy.getRawButton(var.buttonXChannel);  

    // read rb and lb buttons
    boolean buttonRB = Joy.getRawButton(var.buttonRBChannel);
    boolean buttonLB = Joy.getRawButton(var.buttonLBChannel);

    boolean buttonMode = Joy.getRawButton(var.buttonMode); 
    
    // pneumatics control
    // ----------------------------------------------------------------------------------------------------------------------------//

    jawLowerEncoderPosition = (jawLowerEncoder.getPosition());
    jawUpperEncoderPosition = (jawUpperEncoder.getPosition());

    double jawLowerAngle = jawLowerEncoderPosition;  
    double jawUpperAngle = jawUpperEncoderPosition ;

    SmartDashboard.putNumber("Jaw Lower Angle", jawLowerAngle);
    SmartDashboard.putNumber("Jaw Upper Angle", jawUpperAngle);
 
    boolean runIntake = false; 

    // home position 
    if (buttonA) {
      Jaw_Lower_Setpoint = var.homePositionLowerJaw;
      Jaw_Upper_Setpoint = var.homePositionUpperJaw; 
      jawLowerInPosition = false; 
      jawUpperInPosition = false; 
    }

    // shoot position 
    else if(buttonY){
      Jaw_Lower_Setpoint = var.shootPositionLowerJaw;
      Jaw_Upper_Setpoint = var.shootPositionUpperJaw;  
      
      pistonLaunched = true;  
      jawLowerInPosition = false; 
      jawUpperInPosition = false; 
    }
    
    // pick up position 
    else if (buttonB) {
      Jaw_Lower_Setpoint = var.lowerPositionLowerJaw; 
      Jaw_Upper_Setpoint = var.lowerPositionUpperJaw; 
      runIntake = true;
      jawLowerInPosition = false; 
      jawUpperInPosition = false;  
    }


    // jawLowerMotorPIDControllers.setTolerance(1, 10);
    
    double pistonStartTime;

    jawLowerMotor.set(jawLowerMotorPIDControllers.calculate(jawLowerAngle, Jaw_Lower_Setpoint));


    if(Math.abs(Jaw_Lower_Setpoint - jawLowerEncoder.getPosition()) < 2){
        jawUpperMotor.set(jawUpperMotorPIDControllers.calculate(jawUpperAngle, Jaw_Upper_Setpoint));
        
        jawLowerInPosition = true; 


        if(Math.abs(Jaw_Upper_Setpoint - jawUpperEncoder.getPosition()) < 1){

          // put delay here 

          // pistonStartTime = System.currentTimeMillis();
          
          // if (System.currentTimeMillis() - pistonStartTime < var.pistonSlow) {
          //   deployPiston();
          // }

          

          jawUpperInPosition = true; 
          
          
          if(pistonLaunched){
            pistonStartTime = System.currentTimeMillis();

            while (System.currentTimeMillis() - pistonStartTime < var.pistonSlow) {
              deployPiston();
            }

            retractPiston();

            pistonLaunched = false; 

            if(runIntake){
              victor.set(var.intakeLowerSpeed);
            }
          
          }
          
      }
 
      SmartDashboard.putBoolean("piston", pistonLaunched);
      SmartDashboard.putBoolean("jaw lower in position", jawLowerInPosition);
      SmartDashboard.putBoolean("jaw upper in position", jawUpperInPosition);
    }

    // limelight stuff
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    
    // drive train gear box 
    if(buttonRB){
      gearSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    else if(buttonLB){
      gearSolenoid.set(DoubleSolenoid.Value.kReverse);
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


  public void goForward(double setDistance, double distanceTravelled, double gyroSetpoint, double navxYawReading, double distanceError) {


    double initialDrive;

    double gyroError = gyroSetpoint - navxYawReading;

    double gyroDrive = var.gyroKP * gyroError;

    if (distanceError < setDistance * 0.5) {
      initialDrive = 0.75;

    }

    else {
      initialDrive = var.distanceKP * distanceError;
    }

    double leftSpeed = initialDrive + gyroDrive;
    double rightSpeed = initialDrive - gyroDrive;


    // telling the robot to drive
    rightMotorFront.set(rightSpeed);
    leftMotorFront.set(leftSpeed);
  }

  // piston command
  public void deployPiston() {
    pistonSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPiston() {
    pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void offPiston(){
    pistonSolenoid.set(DoubleSolenoid.Value.kOff);
  }

}