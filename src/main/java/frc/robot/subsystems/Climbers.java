package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{
    CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_LEFT_MOTOR,MotorType.kBrushless);
    CANSparkMax rightClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_RIGHT_MOTOR,MotorType.kBrushless);
}
