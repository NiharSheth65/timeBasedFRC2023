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


  // creating enencoder
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder jawLowerEncoder; 
  private RelativeEncoder jawUpperEncoder; 

  // creating compressor object
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // creating double solonoids
  DoubleSolenoid gearSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    DoubleSolenoid pistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  // intial setpoints
  private double LeftMotor_Setpoint = 0;
  private double RightMotor_Setpoint = 0;
  private double Jaw_Lower_Setpoint = 0; 
  private double Jaw_Upper_Setpoint = 0; 

  private double rightEncoderPosition;
  private double leftEncoderPosition;
  private double jawLowerEncoderPosition;
  private double jawUpperEncoderPosition;  

  // actually find the gear ratio and replace, 1 is a place holder
  private double ticksPerFoot = 2;

  // compressor logic
  boolean enabled = pcmCompressor.isEnabled();

  // nav x stuff
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  double navxAutonomousPosition;

//   auto chooser stuff 
// private static final String kMiddleAuto = "middle";
// private static final String kRightAuto = "right";
// private static final String kLeftAuto = "left";
// private String m_autoSelected;
// private final SendableChooser<String> m_chooser = new SendableChooser<>();

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
    // m_chooser.setDefaultOption("Middle Auto", kMiddleAuto);
    // m_chooser.addOption("Right Auto", kRightAuto);
    // m_chooser.addOption("Left Auto", kLeftAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
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
    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {

    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);

     // get encoder readings and convert them to desired units 
     rightEncoderPosition = (leftEncoder.getPosition() / ticksPerFoot);
     leftEncoderPosition = (rightEncoder.getPosition() / ticksPerFoot);
 
     // distance travalled 
     double distanceTravelled = (rightEncoderPosition + leftEncoderPosition)/2; 

     double gyroSetpoint = navxAutonomousPosition; 
     double navxYawReading = navx.getYaw(); 

    //  switch (m_autoSelected) {
    //     case kRightAuto:
    //       // Put custom auto code here
    //       break;
    //     case kLeftAuto: 
    //     // put left code here 
    //       break; 
    //     case kMiddleAuto:
    //     default:
    //       //  should clear doc and point line by this point 
    //         if(distanceTravelled <= 20){
    //         //  goForward(driveSetPoint, distanceTravelled, gyroSetpoint, gyroReading); 
    //             double setDistance = 20; 
    //             goForward(setDistance, distanceTravelled, gyroSetpoint, navxYawReading);  
    //         }

    //       break;
    //   }
    
 

     SmartDashboard.putNumber("right ticks", rightEncoder.getPosition()); 
     SmartDashboard.putNumber("left ticks", leftEncoder.getPosition());
 
     SmartDashboard.putNumber("right voltage", leftMotorFront.getBusVoltage()); 
     SmartDashboard.putNumber("left voltage", leftMotorFront.getBusVoltage()); 


  }

  @Override
  public void teleopInit() {
    navx.reset();
    navx.zeroYaw();
    jawLowerMotor.getEncoder().setPosition(0);
  
    leftMotorFront.getEncoder().setPosition(0);
    rightMotorFront.getEncoder().setPosition(0);
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

    // read rb and lb buttons 
    boolean buttonRB = Joy.getRawButton(var.buttonRBChannel);
    boolean buttonLB = Joy.getRawButton(var.buttonLBChannel);

    // read button right and elft 
    boolean buttonRight = Joy.getRawButton(var.buttonRightChannel); 
    boolean buttonLeft = Joy.getRawButton(var.buttonLeftChannel); 


    // pneumatics control 
    // ----------------------------------------------------------------------------------------------------------------------------//
    if (buttonRB) {
      pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
        SmartDashboard.putString("gear mode", "torque"); 
    }

    else if (buttonLB) {
      pistonSolenoid.set(DoubleSolenoid.Value.kForward);
      SmartDashboard.putString("gear mode", "speed"); 
    }

    // if (buttonRight) {
    //   pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
    // }
  
    // else if (buttonLeft) {
    //    pistonSolenoid.set(DoubleSolenoid.Value.kForward);
    // }


    // --------------------------------------------------------------------------------------------------------------------- //
    // jaw lower and upper stuff 

    jawLowerEncoderPosition = (jawLowerEncoder.getPosition()); 
    jawUpperEncoderPosition = (jawUpperEncoder.getPosition()); 

    double jawLowerAngle = jawLowerEncoderPosition * 7.5; 
    double jawUpperAngle = jawUpperEncoderPosition * 7.5; //double check 7.5 still works here 

    SmartDashboard.putNumber("Jaw Lower Angle", jawLowerAngle);
    SmartDashboard.putNumber("Jaw Upper Angle", jawLowerAngle);


    if (buttonA) {
        Jaw_Lower_Setpoint = 0;  
        Jaw_Upper_Setpoint = 0;
        // jawLowerMotor.set(0.5); 
    }


    else if(buttonB){
        Jaw_Lower_Setpoint = -90; 
        Jaw_Upper_Setpoint = -90; 
    } 

    else {
        jawLowerMotor.set(0);
        jawUpperMotor.set(0); 
    }

    pidLowerJaw(Jaw_Lower_Setpoint, jawLowerAngle); 
    pidUpperJaw(Jaw_Upper_Setpoint, jawUpperAngle); 


    // limelight stuff 
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
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

// lower jaw pid control

  public void pidLowerJaw(double setPoint, double measurment){
    double error = setPoint - measurment; 
    double kp = 0.005; 
 
    // SmartDashboard.putNumber("speed", speed); 

    double speed = error * kp; 
    // double newMeasurment = (jawLowerEncoder.getPosition() * 7.5); 
    // error = setPoint - newMeasurment; 
    SmartDashboard.putNumber("error", error); 
    jawLowerMotor.set(speed);
}

// upper jaw pid control

public void pidUpperJaw(double setPoint, double measurment){
    double error = setPoint - measurment; 
    double kp = 0.005; 
 
    // SmartDashboard.putNumber("speed", speed); 

    double speed = error * kp; 
    // double newMeasurment = (jawLowerEncoder.getPosition() * 7.5); 
    // error = setPoint - newMeasurment; 
    SmartDashboard.putNumber("error", error); 
    jawUpperMotor.set(speed);
}

public void goForward(double setDistance, double distanceTravelled, double gyroSetpoint, double navxYawReading){
     
     double distanceError = setDistance - distanceTravelled;  
      
     double initialDrive; 
     

     double gyroError = gyroSetpoint - navxYawReading; 
     
     double gyroDrive = var.gyroKP * gyroError;  
 
     double leftSpeed; 
     double rightSpeed;  
 

     if(distanceError < setDistance * 0.5){
         initialDrive = 1; 
          
     }
 
     else{
         initialDrive = var.distanceKP * distanceError;
     }
      
     // telling the robot to drive 
     rightMotorFront.set(initialDrive - gyroDrive);    
     leftMotorFront.set(initialDrive + gyroDrive);
}
  
    
    
    // return speed; 

  // work for tommorow, start converting encoder values into degrees
  // use that as a measured value 
  // start determing set point values 
  // try turning to those values 
}