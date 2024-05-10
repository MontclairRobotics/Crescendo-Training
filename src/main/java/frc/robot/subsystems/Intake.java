package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    CANSparkMax topIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_TOP_MOTOR,MotorType.kBrushless);
    CANSparkMax bottomIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
    
    //creates the beambreak as a digitalinput
    DigitalInput beambreak = new DigitalInput(9);

    //True means BB is not broken; false means BB is broken
    public BooleanSupplier getBB(){
        return () -> beambreak.get();
    }
    // Starts intake motors
    public void inhale(){
        
        RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(52));
        topIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
        Transport.start();
       
    }
    public Command inhaleCommand(){
        return Commands.runOnce(()-> inhale(), this).onlyWhile(getBB()).finallyDo(() ->holdBreath());
    }
    // Reverses intake motors
    public void exhale(){
        topIntakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
        Transport.reverse();
    }
    // Stops intake motors
    public void holdBreath(){
        topIntakeMotor.set(0);
        bottomIntakeMotor.set(0);
    
    }
    //command for reverse intaking
    public Command exhaleCommand (){
        return Commands.runOnce(() -> {exhale();}, this).finallyDo(() -> {holdBreath();
        Transport.stop();});
    }
    //command to stop the intake motors
    public Command holdBreathCommand (){
        return Commands.runOnce(() -> {holdBreath();}, this);
    }
}