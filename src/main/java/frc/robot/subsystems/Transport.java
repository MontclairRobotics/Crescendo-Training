package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Transport {
    static CANSparkMax transportMotor = new CANSparkMax(Constants.Ports.SHOOTER_MOTOR_TRANSPORT, MotorType.kBrushless);

    public static void start(){
        transportMotor.set(Constants.TRANSPORT_SPEED);
    }
    public static void reverse(){
        transportMotor.set(-Constants.TRANSPORT_SPEED);
    }
    public static void stop(){
        transportMotor.set(0);
    }
}
