package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climbers extends SubsystemBase{

    CANSparkMax leftMotor = new CANSparkMax(Constants.Ports.CLIMBER_LEFT_MOTOR,MotorType.kBrushless);
    CANSparkMax rightMotor = new CANSparkMax(Constants.Ports.CLIMBER_RIGHT_MOTOR,MotorType.kBrushless);
    DigitalInput leftLimit = new DigitalInput(Constants.Ports.CLIMBER_LEFT_LIMIT_SWITCH_PORT);
    DigitalInput rightLimit = new DigitalInput(Constants.Ports.CLIMBER_RIGHT_LIMIT_SWITCH_PORT);

    boolean rightClimberCanGoDown;
    boolean leftClimberCanGoDown;
    boolean shiftSticks;
    boolean rightGoingUp;
    boolean leftGoingUp;

    double rightInput = 0;
    double leftInput = 0;

    public Climbers() {
        leftMotor.setInverted(true); 
        rightMotor.setInverted(false);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
    public void climbersDefault() {
        if(shiftSticks) leftInput = RobotContainer.operatorController.getLeftY();
        else leftInput = RobotContainer.operatorController.getRightY();
        rightInput = RobotContainer.operatorController.getRightY();
        if(Math.abs(leftInput) < .04) leftInput = 0;
        if(Math.abs(rightInput) < .04) rightInput = 0;
    }
    public Command climbersDefaultCommand(){
        return Commands.run(() -> {climbersDefault();}, this);
    }
    public void moveClimbers(){
        if(leftClimberCanGoDown) leftMotor.set(leftInput);
        else if(leftInput > 0) leftMotor.set(leftInput);
        else stop(leftMotor);
        if(rightClimberCanGoDown) rightMotor.set(rightInput);
        else if(rightInput > 0) rightMotor.set(rightInput);
        else stop(rightMotor);
    }
    public void stop(CANSparkMax motor){
        motor.set(0);
    }
    public void periodic(){

        if (rightLimit.get()) rightClimberCanGoDown = false;
        else rightClimberCanGoDown = true;
        if (leftLimit.get()) leftClimberCanGoDown = false;
        else leftClimberCanGoDown = true;

        shiftSticks = RobotContainer.operatorController.L1().getAsBoolean();

        rightGoingUp = rightInput > 0;  
        leftGoingUp = leftInput > 0;

        moveClimbers();
    
    }
}