// package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase{
    CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_LEFT_MOTOR,MotorType.kBrushless);
    CANSparkMax rightClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_RIGHT_MOTOR,MotorType.kBrushless);
    DigitalInput leftlimitSwitch = new DigitalInput(Constants.Ports.CLIMBER_LEFT_LIMIT_SWITCH_PORT);
    DigitalInput rightlimitSwitch = new DigitalInput(Constants.Ports.CLIMBER_RIGHT_LIMIT_SWITCH_PORT);

    public Climbers(){
        leftClimberMotor.setInverted(true);
        rightClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }
    //start climber motors
    public void start(){
        leftClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
        rightClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
    }

    //stop the climber motors
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }

    //start in the reverse direction
    public void reverse(){
        leftClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        rightClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
    }
    //Start Command
    public Command startCommand(){
        return Commands.runOnce(() -> {start();}, this);
    }
}
