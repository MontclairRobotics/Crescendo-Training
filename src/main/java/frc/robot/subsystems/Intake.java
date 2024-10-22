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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;

public class Intake extends SubsystemBase {
    CANSparkMax topIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_TOP_MOTOR,MotorType.kBrushless);
    CANSparkMax bottomIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
    
    public DigitalInput beambreak = new DigitalInput(Ports.TRANSPORT_BEAM_BREAK);

    //True means BB is not broken; false means BB is broken
    public BooleanSupplier getBB(){
        return () -> beambreak.get();
    }
    public BooleanSupplier getReverseBB(){
        return () -> !beambreak.get();
    }
    // Starts intake motors
    public void intake(){
        RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(ArmConstants.SPROCKET_INTAKE_ANGLE));
        topIntakeMotor.set(IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(IntakeConstants.INTAKE_SPEED);
        Transport.reverse();
    }
    // Reverses intake motors
    public void outtake(){
        RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(ArmConstants.SPROCKET_OUTTAKE_ANGLE));
        topIntakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        Transport.start();
    }
    // Stops intake motors
    public void stop(){
        topIntakeMotor.set(0);
        bottomIntakeMotor.set(0);
        Transport.stop();
    }
    public Command intakeCommand(){
        return Commands.run(()-> intake(), this).onlyWhile(getBB()).finallyDo(() -> stop());
    }
    //command for reverse intaking
    public Command outtakeCommand (){
        return Commands.run(() -> {outtake();}, this).finallyDo(() -> {stop();
        Transport.stop();});
    }
    //command to stop the intake motors
    public Command stopCommand (){
        return Commands.runOnce(() -> {stop();}, this);
    }
}