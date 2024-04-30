package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake {
    CANSparkMax topIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_TOP_MOTOR,MotorType.kBrushless);
    CANSparkMax bottomIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
    
    public void inhale(){
        topIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
    }
    public void exhale(){
        topIntakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
    }
    public void holdBreath(){
        topIntakeMotor.set(0);
        bottomIntakeMotor.set(0);
    }
}