package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class Sprocket extends SubsystemBase {
    
    static CANSparkMax leftSprocketMotor = new CANSparkMax(Constants.Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    static CANSparkMax rightSprocketMotor = new CANSparkMax(Constants.Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    
    boolean canGoUp;
    boolean canGoDown;
    double inputForSprocket;
    boolean isAtSetPoint;
    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3); 

    static PIDController pidController = new PIDController(.8,0,0); 
    static boolean isUsingPID;
    boolean isSprocketSafe;
    
    public Sprocket() {
        absoluteEncoder.setDistancePerRotation(360);
        Shuffleboard.getTab("Debug").addDouble("Sprocket Angle", getRawPositionSupplier());
        leftSprocketMotor.setInverted(true);
        rightSprocketMotor.setInverted(true);
        pidController.setTolerance(1);
    }

    //gets the actual angle of the encoder
    public static double getRawPosition(){
        return absoluteEncoder.getDistance()-281.6;
    }

    //a DoubleSupplier that supplies the actual angle of the encoder for logging purposes
    public DoubleSupplier getRawPositionSupplier(){
        return () -> getRawPosition();
    }

    //method for setting the angle using PID
    public void setAngle(Rotation2d angle){
        isUsingPID = true;
        if(isSprocketSafe) {
        pidController.setSetpoint(angle.getDegrees());
        isUsingPID = true;
        }

    }

    public boolean isAtSetPointMethod(){
        if(pidController.atSetpoint()) return true;
        else return false;
    }

    //command for setting the angle using PID
    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(angle)), this);
    }
    public static void stop() {
        rightSprocketMotor.set(0);
        leftSprocketMotor.set(0);
    }
    public Command stopCommand(){
        return Commands.runOnce(()-> stop());
    }
 
    public Command stopSprocket() {
        return Commands.runOnce(() -> Sprocket.stop(), this);
    }

    public void setBrakeMode(){
        leftSprocketMotor.setIdleMode(IdleMode.kBrake);
        rightSprocketMotor.setIdleMode(IdleMode.kBrake);
    }

    public Command brakeModeCommand(){
        return Commands.runOnce(()-> setBrakeMode(), this);
    }
    

    //moves the sprocket up using the DPAD at a constant speed
    public void up (){
        if(isSprocketSafe && canGoUp){
        isUsingPID = false;
        leftSprocketMotor.set(ArmConstants.SPROCKET_SPEED);
        rightSprocketMotor.set(ArmConstants.SPROCKET_SPEED);
        } else {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        }

    }

    //command for moving sprocket up using DPAD
    public Command upCommand(){
        return Commands.runOnce(() -> up());
    }
    
    //move the sprocket down using the DPAD at a constant speed
    public void down(){
        if(isSprocketSafe && canGoDown){
        isUsingPID = false;
        leftSprocketMotor.set(-ArmConstants.SPROCKET_SPEED);
        rightSprocketMotor.set(-ArmConstants.SPROCKET_SPEED);
        } else {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        }
            

    }

    //commmand for manually moving the sprocket using the DPAD
    public Command downCommand(){
        return Commands.runOnce(() -> down());
    }

    //moves the sprocket based off of joystick input
    //this is the default command

    public void moveSprocket(){
        if(isSprocketSafe){
        leftSprocketMotor.set(inputForSprocket);
        rightSprocketMotor.set(inputForSprocket);
        }
        else if(canGoUp){
        leftSprocketMotor.set(Math.abs(inputForSprocket));
        rightSprocketMotor.set(Math.abs(inputForSprocket));
        } else if(canGoDown) {
        leftSprocketMotor.set(-Math.abs(inputForSprocket));
        rightSprocketMotor.set(-Math.abs(inputForSprocket));
        }
            
        else {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        }

        if(Math.abs(inputForSprocket) < .04){
            isUsingPID = true;
        } else isUsingPID = false;

                

    }

    //command for the moveSprocket method
    public Command moveSprocketCommand(){
        return Commands.runOnce(()->moveSprocket(), this);
    }
  
    public void periodic() {
        if (isUsingPID && isSprocketSafe) {
            leftSprocketMotor.setVoltage(pidController.calculate(getRawPosition()));
            rightSprocketMotor.setVoltage(pidController.calculate(getRawPosition()));
            System.out.println(pidController.calculate(getRawPosition()));
        }
        if (!isSprocketSafe) {
            Sprocket.stop();
            isUsingPID = false;
        }
        // System.out.println("isUSINGPID" + isUsingPID +"  issafe" + isSprocketSafe);
        
     canGoUp = getRawPosition() < (ArmConstants.ENCODER_MAX_ANGLE - 5);
     canGoDown = getRawPosition() > (ArmConstants.ENCODER_MIN_ANGLE + 5);
     isSprocketSafe = getRawPosition() < Constants.ArmConstants.ENCODER_MAX_ANGLE-2 || absoluteEncoder.getAbsolutePosition() > Constants.ArmConstants.ENCODER_MIN_ANGLE+2;
    
    inputForSprocket = RobotContainer.operatorController.getLeftY();
    inputForSprocket = inputForSprocket * inputForSprocket * inputForSprocket;
    
        if(Math.abs(inputForSprocket)<.04){
            inputForSprocket = 0;
            isUsingPID = true;
        } else isUsingPID = false;

    }
}
