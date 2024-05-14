package frc.robot.subsystems;

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

    boolean canRightClimberGoDown = true;
    boolean canLeftClimberGoDown = true;

    public Climbers(){
        leftClimberMotor.setInverted(true); //Do we need this?
        rightClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }
    //start climber motors
    public void goUp(){
        leftClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
        rightClimberMotor.set(Constants.ClimberConstants.CLIMBER_SPEED);
    }
    //stop the climber motors
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
    //start in the reverse direction
    public void goDown(){
        if(canLeftClimberGoDown){
            leftClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        } else {
            leftClimberMotor.set(0);
        }
        if(canRightClimberGoDown){
            rightClimberMotor.set(-Constants.ClimberConstants.CLIMBER_SPEED);
        } else {
            rightClimberMotor.set(0);
        }
    }
    //Start Command
    public Command goUpCommand(){
        return Commands.runOnce(() -> {goUp();}, this);
    }
    //stop Command
    public Command stopCommand(){
        return Commands.runOnce(() -> {stop();}, this);
    }
    //reverse Command
    public Command goDownCommand(){
        return Commands.runOnce(() -> {goDown();}, this);
    }

    public void periodic(){
        if (rightlimitSwitch.get()) {
            canRightClimberGoDown = false;
        } else {
            canRightClimberGoDown = true;
        }
        if (leftlimitSwitch.get()) {
            canLeftClimberGoDown = false;
        } else {
            canLeftClimberGoDown = true;
        }

    }
}
