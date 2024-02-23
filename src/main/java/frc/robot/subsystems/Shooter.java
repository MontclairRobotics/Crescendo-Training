package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.util.Tunable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Drive by scoring angle calculation is: arctan(height/distance);

public class Shooter extends SubsystemBase {

    public final CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TOP_PORT, MotorType.kBrushless);
    public final CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_BOTTOM_PORT, MotorType.kBrushless);
    public final CANSparkMax transportMotor = new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT_PORT, MotorType.kBrushless);

    public Tunable<Double> transportSpeed = Tunable.of(1,"Transport speed");
    public Tunable<Double> speakerShootSpeed = Tunable.of(1, "speaker shoot speed");
    private RelativeEncoder topEncoder = topMotor.getEncoder();
    private RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

    private SparkPIDController topController = topMotor.getPIDController();
    private SparkPIDController bottomController = bottomMotor.getPIDController();


    public Shooter() {
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);
        transportMotor.setInverted(true);

        topController.setI(ShooterConstants.ki.get());
        topController.setP(ShooterConstants.kp.get());
        topController.setD(ShooterConstants.kd.get());
        topController.setFF(ShooterConstants.ff.get());

        bottomController.setI(ShooterConstants.ki.get());
        bottomController.setP(ShooterConstants.kp.get());
        bottomController.setD(ShooterConstants.kd.get());
        bottomController.setFF(ShooterConstants.ff.get());

        ShooterConstants.ki.whenUpdate((ki) -> {
            topController.setI(ki);
            bottomController.setI(ki);
        });

        ShooterConstants.kp.whenUpdate((kp) -> {
            topController.setP(kp);
            bottomController.setP(kp);
        });

        ShooterConstants.kd.whenUpdate((kd) -> {
            topController.setP(kd);
            bottomController.setP(kd);
        });

        ShooterConstants.ff.whenUpdate((ff) -> {
            topController.setFF(ff);
            bottomController.setP(ff);
        });

        topController.setOutputRange(-1, 1);
        bottomController.setOutputRange(-1, 1);
    }
    
    /**
     * Shoots (used for speaker)
     */
    public void shootSpeaker() {
        topMotor.set(speakerShootSpeed.get());
        bottomMotor.set(speakerShootSpeed.get());
        // transportMotor.set(SubsystemConstants.TRANSPORT_SPEED);
    }

    public void shootVelocity(double velocity) {
        topController.setReference(velocity, ControlType.kVelocity);
        bottomController.setReference(velocity, ControlType.kVelocity);

        System.out.println("Top: " + topEncoder.getVelocity() + " Bottom: " + bottomEncoder.getVelocity());
    }

    public boolean isAtSetpoint(double setpoint) {
        return Math.abs(topEncoder.getVelocity() - setpoint) < ShooterConstants.VELOCITY_DEADBAND &&
        Math.abs(bottomEncoder.getVelocity() - setpoint) < ShooterConstants.VELOCITY_DEADBAND;
    }

    public void startTransport() {
        transportMotor.set(transportSpeed.get());
    }
    public void stopTransport() {
        transportMotor.stopMotor();
    }
    public double getTopVelocity() {
        return topEncoder.getVelocity();
    }

    public double getBottomVelocity() {
        return bottomEncoder.getVelocity();
    }
    /**
     * Shoots (used for amp)
     */
    public void shootAmp() {
        topMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
        bottomMotor.set(SubsystemConstants.AMP_EJECT_SPEED);
    }

    /**
     * Stops the motors
     */
    public void stop(){
        topMotor.set(0);
        bottomMotor.set(0);
    }
    /**
     * reverse shooter, in case shooter jams, etc.
     */ 
    public void reverseShooter() {
        topMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
        bottomMotor.set(-ShooterConstants.SPEAKER_EJECT_SPEED);
    }
}