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
        isUsingPID = true;
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
    public void setCoastMode (){
        leftSprocketMotor.setIdleMode(IdleMode.kCoast);
        rightSprocketMotor.setIdleMode(IdleMode.kCoast);
    }
    public Command setCoastModeCommand(){
        return Commands.runOnce(()-> setCoastMode(), this);
    }
    public void moveSprocket(){
        //checks for safeties and if it is safe, it will then set the motors to that speed
        if((!canGoUp && inputForSprocket > 0 ) || (!canGoDown && inputForSprocket < 0)) {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        } else if (Math.abs(inputForSprocket) > 0){
        leftSprocketMotor.set(inputForSprocket);
        rightSprocketMotor.set(inputForSprocket);
        }
    }

    //command for the moveSprocket method
    public Command moveSprocketCommand(){
        return Commands.runOnce(()->moveSprocket(), this);
    }
  
    public Command inhaleSetAngle(){
        return Commands.runOnce(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(52)), this);
    }

    public void periodic() {

        //trying to go up but it cant
        if ((!canGoUp && (pidController.getSetpoint() > 63 ))
        //cant go down but is trying to
        || (!canGoDown && (pidController.getSetpoint() < 26)) ){

            System.out.println("the safety code for PID is running");
            isUsingPID = false;
            stop();

        }
        //if the following statement did not trigger and we are using pid and it is fully safe,
        //or it is at the top limit and trying to go down, or if it as the bottomLimit and trying to go up,
        //it will set the motors to the voltage calculated by PID.
        if (isUsingPID && isSprocketSafe) {
            leftSprocketMotor.setVoltage(pidController.calculate(getRawPosition()));
            rightSprocketMotor.setVoltage(pidController.calculate(getRawPosition()));
        }    

        //safety booleans
        canGoUp = getRawPosition() < (ArmConstants.ENCODER_MAX_ANGLE);
        canGoDown = getRawPosition() > (ArmConstants.ENCODER_MIN_ANGLE);
        isSprocketSafe = getRawPosition() < Constants.ArmConstants.ENCODER_MAX_ANGLE || absoluteEncoder.getAbsolutePosition() > Constants.ArmConstants.ENCODER_MIN_ANGLE;
    
        //calculates the input for the sprocket     
        inputForSprocket = RobotContainer.operatorController.getLeftY();
        inputForSprocket = inputForSprocket * inputForSprocket * inputForSprocket;
    
        if(Math.abs(inputForSprocket)<.04){
            inputForSprocket = 0;
            isUsingPID = true;
            System.out.println("isUSINGPID is being set to true");
        } else isUsingPID = false;
    }
}
