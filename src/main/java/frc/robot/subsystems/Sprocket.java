package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
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

public class Sprocket extends SubsystemBase {
    
    static CANSparkMax leftSprocketMotor = new CANSparkMax(Constants.Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    static CANSparkMax rightSprocketMotor = new CANSparkMax(Constants.Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    
    
    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3); 

    static PIDController pidController = new PIDController(0,0,0); 
    static boolean isUsingPID;
    static boolean isSprocketSafe = getRawPosition() < Constants.ArmConstants.ENCODER_MAX_ANGLE-5 || absoluteEncoder.getAbsolutePosition() > Constants.ArmConstants.ENCODER_MIN_ANGLE+5;

    public Sprocket() {
        absoluteEncoder.setDistancePerRotation(360);
        Shuffleboard.getTab("Debug").addDouble("Sprocket Angle", getRawPositionSupplier());
    }
    public static double getRawPosition(){
        return absoluteEncoder.getDistance()-281.6;
    }
    public static void setAngle(Rotation2d angle){
        pidController.setSetpoint(angle.getDegrees());
        isUsingPID = true;
    }
    public DoubleSupplier getRawPositionSupplier(){
        return () -> getRawPosition();
    }
    public static void stop() {
        rightSprocketMotor.set(0);
        leftSprocketMotor.set(0);
    }
    public Command stopCommand(){
        return Commands.runOnce(()-> stop());
    }
    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> Sprocket.setAngle(Rotation2d.fromDegrees(angle)), this);
    }
    public Command stopSprocket() {
        return Commands.runOnce(() -> Sprocket.stop(), this);
    }

    boolean canGoUp = getRawPosition() < (ArmConstants.ENCODER_MAX_ANGLE - 5);
    boolean canGoDown = getRawPosition() < (ArmConstants.ENCODER_MIN_ANGLE + 5);

    public void setBrakeMode(){
        leftSprocketMotor.setIdleMode(IdleMode.kBrake);
        rightSprocketMotor.setIdleMode(IdleMode.kBrake);
    }
    public Command brakeModeCommand(){
        return Commands.runOnce(()-> setBrakeMode());
    }
    
    public void up (){
        if(isSprocketSafe && canGoUp){
        leftSprocketMotor.set(-.2);
        rightSprocketMotor.set(-.2);
        } else {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        }
    }
    public void down(){
        if(isSprocketSafe && canGoDown){
        leftSprocketMotor.set(.2);
        rightSprocketMotor.set(.2);
        } else {
        leftSprocketMotor.set(0);
        rightSprocketMotor.set(0);
        }
    }
    
    public Command upCommand(){
        return Commands.runOnce(() -> up());
    }
    public Command downCommand(){
        return Commands.runOnce(() -> down());
    }

    public void periodic() {
        // if (isUsingPID && isSprocketSafe) {
        //     leftSprocketMotor.set(pidController.calculate(absoluteEncoder.getDistance()));
        //     rightSprocketMotor.set(pidController.calculate(absoluteEncoder.getDistance()));
        // }
        // if (!isSprocketSafe) {
        //     Sprocket.stop();
        //     isUsingPID = false;
        // }
        

    }
}
