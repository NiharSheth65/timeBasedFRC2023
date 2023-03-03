// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private VictorSP lowerIntakeMotor = new VictorSP(var.lowerIntakePort);
  private VictorSP upperRightIntakeMotor = new VictorSP(var.upperRightIntakePort);
  private VictorSP upperLeftIntakeMotor = new VictorSP(var.upperLeftIntakePort);
  private VictorSP winchMotor = new VictorSP(var.winchPort);

  private DigitalInput limitSwitchBack = new DigitalInput(var.limitSwitchBackPort);
  private DigitalInput limitSwitchFront = new DigitalInput(var.limitSwitcFrontPort);
  private DigitalInput intakeSwitch = new DigitalInput(var.intakeBumperSwitchPort);

  // creating enencoder
  private RelativeEncoder rightFrontEncoder;
  private RelativeEncoder leftFrontEncoder;
  private RelativeEncoder rightBackEncoder;
  private RelativeEncoder leftBackEncoder;

  private RelativeEncoder jawLowerEncoder;
  private RelativeEncoder jawUpperEncoder;

  // creating compressor object
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // creating double solonoids
  DoubleSolenoid gearSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, var.gearSolonoidPortOne,
      var.gearSolonoidPortTwo);
  DoubleSolenoid pistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, var.pistonSolonoidPortOne,
      var.pistonSolonoidPortTwo);

  // intial setpoints
  private double LeftMotor_Setpoint = 0;
  private double RightMotor_Setpoint = 0;
  private double Jaw_Lower_Setpoint = 0;
  private double Jaw_Upper_Setpoint = 0;

  private double rightFrontEncoderPosition;
  private double leftFrontEncoderPosition;
  private double rightBackEncoderPosition;
  private double leftBackEncoderPosition;

  private double jawLowerEncoderPosition;
  private double jawUpperEncoderPosition;

  private boolean limitStateBack;
  private boolean limitStateFront;
  private boolean intakeState;


  double pitchSetpoint; 

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

  PIDController autonDrivePIDControllers = new PIDController(var.drive_kp, var.drive_ki, var.drive_kd);
  PIDController gryoDrivePIDControllers = new PIDController(var.gryo_kp, var.gryo_ki, var.gryo_kd);
  PIDController gryoPitchPIDControllers = new PIDController(var.pitch_kp, var.pitch_ki, var.pitch_kd);

  PIDController velocityRightPIDControllers = new PIDController(var.velocity_kp, var.velocity_ki, var.drive_kd);
  PIDController velocityLeftPIDControllers = new PIDController(var.velocity_kp, var.velocity_ki, var.drive_kd);

  // piston variables
  boolean jawLowerInPosition;
  boolean jawUpperInPosition;
  double lowerIntakeSpeed;
  double upperIntakeSpeed;
  double winchMotorSpeed;

  // mode vairable

  int modeRequested = 0;
  int modeCurrent = 0;

  // upper mode variable

  int setUpperMode = 0;

  // piston stage
  int pistonStage = 0;
  double pistonStartTime;
  double pistonCurrentTime;

  // autonomous mode setpoint
  int autonStage = 0;
  double winchMode;
  double winchStartTime;
  double distanceTravelled;
  double gyroSetpoint;
  double navxYawReading;
  double upperIntakeMode = 0;

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

    // change diretino of the intake motor
    lowerIntakeMotor.setInverted(false);

    // change direction of the upper right intake motor
    upperRightIntakeMotor.setInverted(true);
    upperLeftIntakeMotor.setInverted(false);

    // the back motors will be following the front motors
    leftMotorFront.setInverted(true);
    leftMotorBack.setInverted(true);

    // get back motors to follow front motors
    leftMotorBack.follow(leftMotorFront);
    rightMotorBack.follow(rightMotorFront);

    // setting the encoders
    rightFrontEncoder = rightMotorFront.getEncoder();
    leftFrontEncoder = leftMotorFront.getEncoder();
    rightBackEncoder = rightMotorBack.getEncoder();
    leftBackEncoder = leftMotorBack.getEncoder();

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

    // jawLowerMotor.set(jawLowerMotorPIDControllers.calculate(jawLowerEncoder.getPosition(),
    // 0));
    // jawUpperMotor.set(jawUpperMotorPIDControllers.calculate(jawUpperEncoder.getPosition(),
    // 0));

    jawLowerInPosition = true;
    jawUpperInPosition = true;

    offPiston();
    jawLowerMotorPIDControllers.reset();
    jawUpperMotorPIDControllers.reset();

    modeRequested = 1;

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
    pitchSetpoint = navx.getRoll(); 

    // option chooser
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    winchMode = 0;
    autonStage = 0;
  }

  @Override
  public void autonomousPeriodic() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

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
        autoSimple();
    }

    SmartDashboard.putNumber("right ticks", rightFrontEncoder.getPosition());
    SmartDashboard.putNumber("left ticks", leftFrontEncoder.getPosition());

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
    leftMotorBack.getEncoder().setPosition(0);
    rightMotorBack.getEncoder().setPosition(0);

    jawLowerMotor.getEncoder().setPosition(0);
    jawUpperMotor.getEncoder().setPosition(0);

    modeRequested = 1;
    setUpperMode = 0;
    winchMode = 0;
    jawLowerMotorPIDControllers.reset();
    jawUpperMotorPIDControllers.reset();
    autonDrivePIDControllers.reset();
    gryoDrivePIDControllers.reset();
    gryoPitchPIDControllers.reset();

  }

  @Override
  public void teleopPeriodic() {

    rightFrontEncoderPosition = rightFrontEncoder.getPosition();
    leftFrontEncoderPosition = leftFrontEncoder.getPosition();
    SmartDashboard.putNumber("right encoder position teleOp", rightFrontEncoderPosition);
    SmartDashboard.putNumber("left encoder position teleOp", leftFrontEncoderPosition);

    // get joy stick readings
    double joyStick_left_Y = -Joy.getRawAxis(var.joyStickLeft_AxisY) * var.safetyFactor;
    double joyStick_right_X = -Joy.getRawAxis(var.joyStickRight_AxisX);
    double robotTurnControl;

    if (joyStick_left_Y == 0) {
      robotTurnControl = 0.5;
    }

    else {
      robotTurnControl = var.turnRate;
    }

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

    boolean rightJoyButton = Joy.getRawButton(var.rightJoyStickButton);
    boolean leftJoyButton = Joy.getRawButton(var.leftJoyStickButton);

    // triggers
    double leftTriggerValue = Joy.getRawAxis(var.leftTriggerButton);
    double rightTriggerValue = Joy.getRawAxis(var.rightTriggerButton);

    SmartDashboard.putNumber("left trigger", leftTriggerValue);
    SmartDashboard.putNumber("right trigger", rightTriggerValue);

    // drive train gear box
    if (rightJoyButton) {
      gearSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    else if (leftJoyButton) {
      gearSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // SmartDashboard.putNumber("upper mode", setUpperMode);
    // SmartDashboard.putNumber("right front encoder values", rightFrontEncoder.getPosition());
    // SmartDashboard.putNumber("right back encoder values", rightBackEncoder.getPosition());
    // SmartDashboard.putNumber("left front encoder values", leftFrontEncoder.getPosition());
    // SmartDashboard.putNumber("left back encoder values", leftBackEncoder.getPosition());

    // SmartDashboard.putBoolean("FRONT LIMIT SWITCH", limitStateFront);
    // SmartDashboard.putBoolean("BACK LIMIT SWITCH", limitStateBack);
    // SmartDashboard.putBoolean("INTAKE LIMIT SWITCH", intakeState);

    // SmartDashboard.putNumber("right velocity", rightFrontEncoder.getVelocity());
    // SmartDashboard.putNumber("left velocity", leftFrontEncoder.getVelocity());

    // uper intake control
    if (buttonRB) {
      upperIntakeMode = 2;
    }

    else if (buttonLB) {
      upperIntakeMode = 1;
    }

    else {
      upperIntakeMode = 0;
    }
    // winch control
    if (leftTriggerValue >= 0.5) {
      winchMode = 2;
    }

    else if (rightTriggerValue >= 0.5) {
      winchMode = 1;
    }

    // else{
    // winchMode = 0;
    // }

    winchControl();
    upperIntakeControl();

    // ----------------------------------------------------------------------------------------------------------------------------//

    // home position
    if (buttonA) {
      modeRequested = 1;
    }

    // shoot position
    else if (buttonY) {
      modeRequested = 2;
    }

    // pick up position
    else if (buttonB) {
      modeRequested = 3;
    }

    // cone pick up position
    else if (buttonX) {
      modeRequested = 4;
    }

    modeCurrent = changeMode(modeRequested, modeCurrent);
    runModes();

    SmartDashboard.putNumber("pitch value", navx.getPitch());

    // jawLowerMotorPIDControllers.setTolerance(1, 10);

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
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("gyro pitch", navx.getPitch()); 
    SmartDashboard.putNumber("gyro roll", navx.getRoll()); 
    SmartDashboard.putNumber("gyro yaw", navx.getYaw()); 
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

  // piston command
  public void deployPiston() {
    pistonSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPiston() {
    pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void offPiston() {
    pistonSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  // mode stuff

  public void runModes() {
    // variable declrations

    double jawLowerAngle = jawLowerEncoderPosition;
    double jawUpperAngle = jawUpperEncoderPosition;

    SmartDashboard.putNumber("Jaw Lower Angle", jawLowerAngle);
    SmartDashboard.putNumber("Jaw Upper Angle", jawUpperAngle);

    // jawLowerMotor.set(jawLowerMotorPIDControllers.calculate(jawLowerAngle,
    // Jaw_Lower_Setpoint));

    lowerIntakeMotor.set(lowerIntakeSpeed);

    if (Math.abs(Jaw_Lower_Setpoint - jawLowerEncoder.getPosition()) < 2) {
      // jawUpperMotor.set(jawUpperMotorPIDControllers.calculate(jawUpperAngle,
      // Jaw_Upper_Setpoint));

      jawLowerInPosition = true;

      if (Math.abs(Jaw_Upper_Setpoint - jawUpperEncoder.getPosition()) < 1) {

        jawUpperInPosition = true;
        pistonControl();
        autonStage++;
      }

      SmartDashboard.putBoolean("jaw lower in position", jawLowerInPosition);
      SmartDashboard.putBoolean("jaw upper in position", jawUpperInPosition);
    }

  }

  public int changeMode(int newModeSelected, int newModeCurrent) {

    // add more mdos here
    // check to see if modes are same,
    // only run if rquests and current are not equal

    if (newModeCurrent != newModeSelected) {
      // mode 1 home positio

      if (newModeSelected == 1) {
        Jaw_Lower_Setpoint = var.homePositionLowerJaw;
        Jaw_Upper_Setpoint = var.homePositionUpperJaw;
        jawLowerInPosition = false;
        jawUpperInPosition = false;
        lowerIntakeSpeed = 0;
        pistonStage = 0;
        retractPiston();
      }

      // mode 2 shoot position and shoot piston
      else if (newModeSelected == 2) {
        Jaw_Lower_Setpoint = var.shootPositionLowerJaw;
        Jaw_Upper_Setpoint = var.shootPositionUpperJaw;
        jawLowerInPosition = false;
        jawUpperInPosition = false;
        lowerIntakeSpeed = 0;
        pistonStage = 1;
      }

      // mode 3 pick up position

      else if (newModeSelected == 3) {
        Jaw_Lower_Setpoint = var.pickUpPositionLowerJaw;
        Jaw_Upper_Setpoint = var.pickUpPositionUpperJaw;
        jawLowerInPosition = false;
        jawUpperInPosition = false;
        lowerIntakeSpeed = -0.2;
        pistonStage = 0;
        retractPiston();
      }

      // mode 4 autonomous position
      // not sure what will go in here yet

      // mode 2 shoot position and shoot piston
      else if (newModeSelected == 4) {
        Jaw_Lower_Setpoint = var.shootPositionLowerJaw;
        Jaw_Upper_Setpoint = var.pickUpPositionUpperJaw;
        jawLowerInPosition = false;
        jawUpperInPosition = false;
        lowerIntakeSpeed = 0;
        pistonStage = 0;
        retractPiston();
      }

      else if (newModeSelected == 5) {
        Jaw_Lower_Setpoint = var.pickUpPositionLowerJaw;
        Jaw_Upper_Setpoint = var.pickUpPositionUpperJawConeClamp;
        jawLowerInPosition = false;
        jawUpperInPosition = false;
        lowerIntakeSpeed = -0.2;
        pistonStage = 0;
        retractPiston();
      }

      return newModeSelected;
    }

    return newModeCurrent;
  }

  public void upperModeStuff() {
    System.out.print(setUpperMode);

    if (setUpperMode == 0) {
      upperIntakeSpeed = 0;
      winchMotorSpeed = 0;
    }

    // intake
    else if (setUpperMode == 1) {
      upperIntakeSpeed = var.intakeUpperSpeed;
      winchMotorSpeed = 0;
    }

    // outtake
    else if (setUpperMode == 2) {
      upperIntakeSpeed = -(var.intakeUpperSpeed);
      winchMotorSpeed = 0;
    }

    else if (setUpperMode == 3) {
      upperIntakeSpeed = 0;
      winchMotorSpeed = var.winchSpeed;

    }

    else if (setUpperMode == 4) {
      upperIntakeSpeed = 0;
      winchMotorSpeed = -(var.winchSpeed);
    }

    upperLeftIntakeMotor.set(upperIntakeSpeed);
    upperRightIntakeMotor.set(upperIntakeSpeed);
    winchMotor.set(winchMotorSpeed);
  }

  public void pistonControl() {
    if (pistonStage == 1) {
      pistonStartTime = System.currentTimeMillis();
      pistonStage++;
      offPiston();
    }

    else if (pistonStage == 2) {
      if (System.currentTimeMillis() > pistonStartTime + 500) {
        pistonStage++;
        deployPiston();
        pistonStartTime = System.currentTimeMillis();
      }
    }

    else if (pistonStage == 3) {
      if (System.currentTimeMillis() > pistonStartTime + 500) {
        pistonStage = 0;
        retractPiston();
        modeRequested = 4;
      }
    }
  }

  // drop cone off in co-opertition zone
  // 1. winch out
  // 2. outtake
  // 3. winch back in
  // 4. drive backwards
  // 5. clear the line
  // 6. drive forward on to dock
  // 7. balance on dock

  public void autoMiddle() {
    // take winch out
    if (autonStage == 0) {
      winchMode = 1;
      winchControl();
    }

    // out take preload
    else if (autonStage == 1) {
      upperIntakeMode = 2;
      upperIntakeControl();
    }

    // bring winch in
    else if (autonStage == 2) {
      winchMode = 2;
      winchControl();
    }

    // 2.26 rev per foot
    // drive forward roughly 13 feet
    else if (autonStage == 3) {
      goForward(30);
    }

    else if (autonStage == 4) {
      goForward(-5.65);
    }

    else if (autonStage == 5) {
      balanceOnDock();
    }

    SmartDashboard.putNumber("auton stage", autonStage);
  }

  public void autoSimple() {
    // take winch out
    double autonStartTime = 0;
    double goForwardDistance = 0;

    if (autonStage == 0) {
      winchMode = 1;
    }

    // out take preload
    else if (autonStage == 1) {
      winchMode = 0;
      upperIntakeMode = 3;
      autonStartTime = System.currentTimeMillis();
      autonStage++;
    }

    else if (autonStage == 2) {
      if (System.currentTimeMillis() > (autonStartTime + 5000)) {
        autonStage++;
      }
    }

    // bring winch in
    else if (autonStage == 3) {
      winchMode = 2;
    }

    else if (autonStage == 4) {
      goForward(var.distanceToClearCommunityOnTheSide);
    }

    else if (autonStage == 5) {
      winchMode = 0;
      upperIntakeMode = 0;
    }

    winchControl();
    upperIntakeControl();
    // drive forward roughly 20 feet
    // else if(autonStage == 4){
    // goForward(-37);
    // }

    SmartDashboard.putNumber("auton stage", autonStage);
  }

  public void goForward(double driveForwardSetPoint) {

    double leftWheelVelocity = leftFrontEncoder.getVelocity();
    double rightWheelVelocity = rightFrontEncoder.getVelocity();

    double maxVelocity = 200;

    double autonomousSpeed;
    double rightWheelOutput;
    double leftWheelOutput;

    double gyroSpeed = gryoDrivePIDControllers.calculate(navx.getYaw(), navxAutonomousPosition);

    // sign should otherwise be flipped to >
    if (rightFrontEncoder.getPosition() > driveForwardSetPoint) {
      if (driveForwardSetPoint < 0) {
        autonomousSpeed = -var.autonomousSpeedFast;
      }

      else {
        autonomousSpeed = var.autonomousSpeedFast;
      }
    }

    else {
      autonomousSpeed = 0;
      gyroSpeed = 0; 
      autonStage++;
    }

    rightWheelOutput = velocityRightPIDControllers.calculate(rightWheelVelocity, autonomousSpeed);
    leftWheelOutput = velocityLeftPIDControllers.calculate(leftWheelVelocity, autonomousSpeed);

    double maxOutput = 0.8;

    if (leftWheelOutput > maxOutput) {
      leftWheelOutput = maxOutput;
    }

    else if (leftWheelOutput < -maxOutput) {
      leftWheelOutput = -maxOutput;
    }

    if (rightWheelOutput > maxOutput) {
      rightWheelOutput = maxOutput;
    }

    else if (rightWheelOutput < -maxOutput) {
      leftWheelOutput = -maxOutput;
    }

    // adding some gyro stuff
    

    rightMotorFront.set(rightWheelOutput - gyroSpeed);
    leftMotorFront.set(leftWheelOutput + gyroSpeed);

    // rightMotorFront.set(rightWheelOutput);
    // leftMotorFront.set(leftWheelOutput);
  }

  public void turn(double targetGyroValue) {
    double gyroPosition = navx.getAngle();
    double calculatedOutPutSpeed = gryoDrivePIDControllers.calculate(navxYawReading, targetGyroValue);

    leftMotorFront.set(calculatedOutPutSpeed);
    rightMotorFront.set(-calculatedOutPutSpeed);

    if (Math.abs(targetGyroValue - gyroPosition) < 0.05) {
      autonStage++;
    }

  }

  public void balanceOnDock() {
    double gyroPitch = navx.getPitch();
    double gyroSetPoint = pitchSetpoint;

    double calculatedBalanceSpeed = gryoPitchPIDControllers.calculate(gyroPitch, pitchSetpoint);
    leftMotorFront.set(calculatedBalanceSpeed);
    rightMotorBack.set(calculatedBalanceSpeed);
  }

  public void winchControl() {

    limitStateBack = limitSwitchBack.get();
    limitStateFront = limitSwitchFront.get();
    // intakeState = intakeSwitch.get();

    // button rb is asking to go forward, give forward speed
    if (winchMode == 0) {
      winchMotorSpeed = 0;
    }

    else if (winchMode == 1) {
      if (limitStateFront == true) {
        winchMotorSpeed = 0;
        winchMode = 0;
        autonStage++;
      }

      else {
        winchMotorSpeed = -var.winchSpeed;
      }
    }

    // button lb is asking to go backward, give backward speed
    else if (winchMode == 2) {
      if (limitStateBack == true) {
        winchMotorSpeed = 0;
        winchMode = 0;
        autonStage++;
      }

      else {
        winchMotorSpeed = var.winchSpeed;
      }
    }

    else {
      winchMotorSpeed = 0;
    }

    // if back end has been reached, cut off motor
    // by default, limitBakc will probably be true

    // if front end has been reached cut off motor
    // motor has reached far end, requesting to go mor for forward
    winchMotor.set(winchMotorSpeed);

    SmartDashboard.putBoolean("limit state back", limitStateBack);
    SmartDashboard.putBoolean("limit state front", limitStateFront);

  }

  public void upperIntakeControl() {

    intakeState = intakeSwitch.get();
    double intakeUpperMechanismSpeed = 0;

    if (upperIntakeMode == 1) {
      // setUpperMode = 1;
      if (intakeState == false) {
        intakeUpperMechanismSpeed = 0.05;
      }

      else {
        intakeUpperMechanismSpeed = var.outtakeUpperSpeed;
      }

    }

    // rb = 2
    // lb = 1
    else if (upperIntakeMode == 2) {
      intakeUpperMechanismSpeed = -var.outtakeUpperSpeed;
      // if(intakeState == false){
      // autonStage++;
      // }
    }

    // auto mode don't use in tele op
    else if (upperIntakeMode == 3) {
      intakeUpperMechanismSpeed = -var.autoOuttakeUpperSpeed;
      // if(intakeState == false){
      // autonStage++;
      // }
    }

    else {
      intakeUpperMechanismSpeed = 0.05;
    }

    upperRightIntakeMotor.set(intakeUpperMechanismSpeed);
    upperLeftIntakeMotor.set(intakeUpperMechanismSpeed);

    SmartDashboard.putBoolean("intake upper state", intakeState);
  }
}