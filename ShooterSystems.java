package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;

public class ShooterSystems {

    Joystick leftJoy, rightJoy; 
    public static WPI_TalonFX intake, shooter;

    public ShooterSystems(Joystick leftJoy, Joystick rightJoy) {
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
        intake = new WPI_TalonFX(2);
        intake.set(ControlMode.PercentOutput, 0);
        shooter = new WPI_TalonFX(0);
        shooter.set(ControlMode.PercentOutput, 0);


    }

    void run() {
        //Intake

    if (leftJoy.getRawButtonPressed(1)) {
        intake.set(ControlMode.PercentOutput, -1);
    } 
    if (leftJoy.getRawButtonReleased(1)) {
        intake.set(ControlMode.PercentOutput, 0);
    }
    if (leftJoy.getRawButtonPressed(4)) {
        intake.set(ControlMode.PercentOutput, 1);
    }
    if (leftJoy.getRawButtonReleased(4)){
        intake.set(ControlMode.PercentOutput, 0);
    }
    if (leftJoy.getRawButtonPressed(6)) {
        intake.set(ControlMode.PercentOutput, 0.25);
    }
    if (leftJoy.getRawButtonReleased(6)) {
        intake.set(ControlMode.PercentOutput, 0);
    }
    
    //Shooter
    
    if (rightJoy.getRawButtonPressed(1)) {
    shooter.set(ControlMode.PercentOutput, -1);
    } 
    if (rightJoy.getRawButtonReleased(1)) {
    shooter.set(ControlMode.PercentOutput, 0);
    }
    }
    
}
