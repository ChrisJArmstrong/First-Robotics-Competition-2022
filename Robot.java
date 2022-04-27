// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  MyRobotDriveSystem myDrive;
  ShooterSystems  mSystems;
  WPI_TalonFX intake, shooter;
  DoubleSolenoid leftActuator, rightActuator; 
  MyRobotDriveSystem mDriveSystem;
  ShooterSystems  mShooterSystems;
  private double startTime;
  UsbCamera camera1;
  UsbCamera camera2;
  NetworkTableInstance cameraSelection;
  public Joystick leftJoy, rightJoy;
  Compressor pcmCompressor;
  AnalogPotentiometer pressureTransducer;
  



  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    leftJoy = new Joystick(0);
    rightJoy = new Joystick(1);


    myDrive = new MyRobotDriveSystem(leftJoy, rightJoy);
    mSystems = new ShooterSystems(leftJoy, rightJoy);


    //Pnumatics 

    leftActuator = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
    rightActuator = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 6, 7);

    leftActuator.set(Value.kForward);
    rightActuator.set(Value.kReverse);

    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    pcmCompressor.enableDigital();




    int scale = 250, offset = -25;
    pressureTransducer = new AnalogPotentiometer(3, scale, offset);



    //Camera

    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    //Shooter

    shooter = ShooterSystems.shooter;
    intake = ShooterSystems.intake;


  }



  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() { 
    startTime = Timer.getFPGATimestamp();
  }
    

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  double time = Timer.getFPGATimestamp();

  pcmCompressor.disable();


  if (time - startTime < 3) {
  myDrive.leftFrontMotor.set(ControlMode.PercentOutput, .25);
  myDrive.leftRearMotor.set(ControlMode.PercentOutput, .25);
  myDrive.rightFrontMotor.set(ControlMode.PercentOutput, .25);
  myDrive.rightRearMotor.set(ControlMode.PercentOutput, .25);
  } else {
  myDrive.leftFrontMotor.set(ControlMode.PercentOutput, 0);
  myDrive.leftRearMotor.set(ControlMode.PercentOutput, 0);
  myDrive.rightFrontMotor.set(ControlMode.PercentOutput, 0);
  myDrive.rightRearMotor.set(ControlMode.PercentOutput, 0);
  }


  
  if (time - startTime < 10) {
    shooter.set(ControlMode.PercentOutput, -1);
  } 
  
  if (time - startTime > 15) {
    shooter.set(ControlMode.PercentOutput, 0);
  }


  if (time - startTime > 4) {
    intake.set(ControlMode.PercentOutput, -1);
  } 
  
  if (time - startTime > 10) {
    intake.set(ControlMode.PercentOutput, 0);
  }

  

  
 

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    mSystems.run();
    myDrive.run();
    
        

  //Pnumatics

  if (rightJoy.getRawButtonPressed(12)) {
    rightActuator.set(DoubleSolenoid.Value.kForward);
    leftActuator.set(DoubleSolenoid.Value.kReverse);
  }
  
  if (rightJoy.getRawButtonPressed(11)) {
    rightActuator.set(DoubleSolenoid.Value.kReverse);
    leftActuator.set(DoubleSolenoid.Value.kForward);  
  }

  if (rightJoy.getRawButtonPressed(7)) {
    pcmCompressor.enableDigital();
  }
  if (rightJoy.getRawButtonPressed(8)) {
    pcmCompressor.disable();
  }

  double psi = pressureTransducer.get();
  SmartDashboard.putNumber("PSI", psi);

  if (psi >=110) {
    pcmCompressor.disable();
  }

  //Limelight

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ty = table.getEntry("ty");
  double cameraAngle = 50;
  double goalHeight = 104;
  double cameraHeight = 26;
  double targetAngle = ty.getDouble(0.0);

      
  
  double a1 = Math.toRadians(cameraAngle);
  double a2 = Math.toRadians(targetAngle);
  double netHeight = goalHeight - cameraHeight;
  double distance = (netHeight) / Math.abs(Math.tan(a1 + a2)); 
  double td = (distance / 12) * 1.9;

  SmartDashboard.putNumber("Distance", td);
  

  /*
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry ty = table.getEntry("ty");
double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how many degrees back is your limelight rotated from perfectly vertical?
double limelightMountAngleDegrees = 50.0;

// distance from the center of the Limelight lens to the floor
double limelightLensHeightInches = 26.0;

// distance from the target to the floor
double goalHeightInches = 102.0;

double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

SmartDashboard.putNumber("Distance", distanceFromLimelightToGoalInches);
*/


  
 

}







  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}