package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sprocket extends SubsystemBase {
    static CANSparkMax leftSprocketMotor = new CANSparkMax(Constants.Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    static CANSparkMax rightSprocketMotor = new CANSparkMax(Constants.Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    
    
    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(3); 

    static PIDController pidController = new PIDController(0,0,0); 
    static boolean isUsingPID;
    static boolean isSprocketSafe = absoluteEncoder.getDistance() < Constants.ArmConstants.ENCODER_MAX_ANGLE-2 || absoluteEncoder.getAbsolutePosition() > Constants.ArmConstants.ENCODER_MIN_ANGLE+2;

    public Sprocket() {
        absoluteEncoder.setDistancePerRotation(360);
    }

    public static void setAngle(Rotation2d angle){
        pidController.setSetpoint(angle.getDegrees());
        isUsingPID = true;
    }
    public static void stop() {
        rightSprocketMotor.set(0);
        leftSprocketMotor.set(0);
    }
    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> Sprocket.setAngle(Rotation2d.fromDegrees(angle)), this);
    }
    public Command stopSprocket() {
        return Commands.runOnce(() -> Sprocket.stop(), this);
    }
    
    public void periodic() {
        if (isUsingPID && isSprocketSafe) {
            leftSprocketMotor.set(pidController.calculate(absoluteEncoder.getDistance()));
            rightSprocketMotor.set(pidController.calculate(absoluteEncoder.getDistance()));
        }
        if (!isSprocketSafe) {
            Sprocket.stop();
            isUsingPID = false;
        }
        

    }
}

