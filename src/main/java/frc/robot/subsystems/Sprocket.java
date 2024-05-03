package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sprocket{
    static CANSparkMax leftSprocketMotor = new CANSparkMax(Constants.Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    static CANSparkMax rightSprocketMotor = new CANSparkMax(Constants.Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    
    //TODO: Actually Use These
    RelativeEncoder leftEncoder = leftSprocketMotor.getEncoder(); 
    RelativeEncoder rightEncoder = rightSprocketMotor.getEncoder(); 

    static PIDController pidController = new PIDController(1,1,1); 
    
    public static void SprocketUp(){
    } 
    public static void SprocketDown(){   
    }
    public static void setAngle(Rotation2d angle){
        pidController.setSetpoint(angle.getDegrees());
    }
    public static void stop(){
        rightSprocketMotor.set(0);
        leftSprocketMotor.set(0);
    }
    public Command setAngleCommand(double angle){
        return Commands.runOnce(() -> Sprocket.setAngle(Rotation2d.fromDegrees(angle)));
    }
    public Command stopSprocket(){
        return Commands.runOnce(() -> Sprocket.stop());
    }
    public void periodic(){
    }
}
