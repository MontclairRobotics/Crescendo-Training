package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class Intake {
    CANSparkMax topIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_TOP_MOTOR,MotorType.kBrushless);
    CANSparkMax bottomIntakeMotor = new CANSparkMax(Constants.Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
    
    //creates the beambreak as a digitalinput
    DigitalInput beambreak = new DigitalInput(9);

 
    //True means BB is not broken; false means BB is broken (NOT HOW MOST THINGS WORK OUR THING IS WEIRD - JAMES RECHS)
    public BooleanSupplier getBB(){
        return () -> beambreak.get();
    }
    // Starts intake motors
    public void inhale(){
        topIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
        bottomIntakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
        Transport.start();
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
    public Command inhaleCommand (){
        return Commands.run(() -> {inhale();}).onlyWhile(getBB());
    }
    public Command exhaleCommand (){
        return Commands.run(() -> {exhale();});
    }
    public Command holdBreathCommand (){
        return Commands.run(() -> {holdBreath();});
    }
}