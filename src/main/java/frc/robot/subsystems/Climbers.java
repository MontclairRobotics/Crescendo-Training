package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climbers extends SubsystemBase{

    CANSparkMax leftClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_LEFT_MOTOR,MotorType.kBrushless);
    CANSparkMax rightClimberMotor = new CANSparkMax(Constants.Ports.CLIMBER_RIGHT_MOTOR,MotorType.kBrushless);
    DigitalInput leftlimitSwitch = new DigitalInput(Constants.Ports.CLIMBER_LEFT_LIMIT_SWITCH_PORT);
    DigitalInput rightlimitSwitch = new DigitalInput(Constants.Ports.CLIMBER_RIGHT_LIMIT_SWITCH_PORT);

    boolean canRightClimberGoDown;
    boolean canLeftClimberGoDown;

    double rightInput = 0;
    double leftInput = 0;

    public void climbersDefault() {
        if(RobotContainer.shouldShiftClimberDefaultCommand){
        rightInput = RobotContainer.operatorController.getRightY();
        leftInput = RobotContainer.operatorController.getLeftY();
        } else {
        rightInput = RobotContainer.operatorController.getRightY();
        leftInput = RobotContainer.operatorController.getRightY();
        }
    }
    public Command DefaultCommand(){
        return Commands.runOnce(() -> {climbersDefault();}, this);
    }
    public Climbers() {
        leftClimberMotor.setInverted(true); //we might not need this
        rightClimberMotor.setInverted(true);
        leftClimberMotor.setIdleMode(IdleMode.kBrake);
        rightClimberMotor.setIdleMode(IdleMode.kBrake);
    }
    public void moveRight(double input){
        if(Math.abs(input) < .05 || (input < 0 && !canRightClimberGoDown)) stop();
        else rightClimberMotor.set(input);
    }
    public void moveLeft(double input){
        if(Math.abs(input) < .05 || (input < 0 && !canLeftClimberGoDown)) stop();
        else leftClimberMotor.set(input);
    }
    public void stop(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
    public void periodic(){

        if (rightlimitSwitch.get()) canRightClimberGoDown = false;
        else canRightClimberGoDown = true;
        if (leftlimitSwitch.get()) canLeftClimberGoDown = false;
        else canLeftClimberGoDown = true;

        moveLeft(leftInput);
        moveRight(rightInput);
        
    }
}
