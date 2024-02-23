package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.util.Tunable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax topMotor = new CANSparkMax(Ports.INTAKE_TOP_MOTOR, MotorType.kBrushless); 
    private final CANSparkMax bottomMotor = new CANSparkMax(Ports.INTAKE_BOTTOM_MOTOR, MotorType.kBrushless);
    //private final DigitalInput beamBreak = new DigitalInput(Ports.BEAM_BREAK_CHANNEL);
    public Tunable<Double> intakeSpeed = Tunable.of(1, "Intake Speed");
    /**
     * Accelerates motors to intake something
     */
    public void in() {
        topMotor.set(-intakeSpeed.get()); 
        bottomMotor.set(-intakeSpeed.get());
    }
    /**
     * Reverse intake if gamepiece gets stuck
     */
    public void out() {
        topMotor.set(intakeSpeed.get());
        bottomMotor.set(intakeSpeed.get());
    }
    
    /**
     * Stop intaking
     */
    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
    }

    /**
     * gets the state of the beam break sensor in the intake
     * @return true if the sensor is broken (gamepiece intaked), false if unbroken
     */
    // public boolean getSensor() {
    //     return !beamBreak.get();
    // }

}