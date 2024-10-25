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

    public boolean bbTriggered;
    public boolean bbNotTriggered;

    public BooleanSupplier noteInTransport(){
        return () -> bbTriggered;
    }
    public BooleanSupplier noteOutOfTransport(){
        return () -> bbNotTriggered;
    }
   
    public void start(){
        topIntakeMotor.set(IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(IntakeConstants.INTAKE_SPEED);
        Transport.start();
    }

    public void reverse(){
        topIntakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        Transport.reverse();
    }
    
    public void stop(){
        topIntakeMotor.set(0);
        bottomIntakeMotor.set(0);
        Transport.stop();
    }

    public void periodic(){
        bbTriggered = !beambreak.get();
        bbNotTriggered = beambreak.get();
    }

}