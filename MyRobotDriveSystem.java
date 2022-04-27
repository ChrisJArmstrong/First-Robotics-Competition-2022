package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class MyRobotDriveSystem {
    
    Joystick leftJoy, rightJoy;
    WPI_TalonSRX leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor;
    MotorControllerGroup m_left, m_right;
    DifferentialDrive m_drive;

    public MyRobotDriveSystem(Joystick leftJoy, Joystick rightJoy){
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
        leftFrontMotor = new WPI_TalonSRX(6); 
        rightFrontMotor = new WPI_TalonSRX(7);
        leftRearMotor = new WPI_TalonSRX(3); 
        rightRearMotor = new WPI_TalonSRX(4);
    
    
        m_left = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
        m_right = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
    
        m_drive = new DifferentialDrive(m_left, m_right);
    
        
        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);
    
    }

    void run(){
        m_drive.tankDrive(leftJoy.getY()*.8, rightJoy.getY()*.8);
    } //                                   ^                    ^
    //To increase speed, just change these values
}
