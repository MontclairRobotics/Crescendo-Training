package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Transport {
    CANSparkMax transportMotor = new CANSparkMax(Constants.Ports.SHOOTER_MOTOR_TRANSPORT, MotorType.kBrushless);

    public void start(){
        transportMotor.set(Constants.TRANSPORT_SPEED);
    }
    public void stop(){
        transportMotor.set(0);
    }
}
