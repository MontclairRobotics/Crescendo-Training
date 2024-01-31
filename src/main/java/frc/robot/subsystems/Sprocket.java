package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.PIDMechanism;
import frc.robot.Constants.*;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax motor = new CANSparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);
    public final PIDController pidController = new PIDController(SubsystemConstants.angleKP, SubsystemConstants.angleKI, SubsystemConstants.angleKD);
    public final PIDMechanism pid;
    private final double speed = SubsystemConstants.ANGLE_SPEED;
    RelativeEncoder encoder;

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(SubsystemConstants.BOTTOM_LIMIT_SWITCH, false);
    public LimitSwitch topLimitSwitch = new LimitSwitch(SubsystemConstants.BOTTOM_LIMIT_SWITCH, false);

    public Sprocket() {

        motor.setInverted(false);

        pid = new PIDMechanism(pidController);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(SubsystemConstants.SPROCKET_ROTATIONS_PER_DEGREE);
        encoder.setPosition(SubsystemConstants.ENCODER_MIN_ANGLE);
    }
    // Move sprocket up
    public void goUp() {
        pid.setSpeed(speed);
    }
    // Move sprocket down
    public void goDown() {
        pid.setSpeed(-speed);
    }
    // Stop sprocket
    public void stop() {
        pid.setSpeed(0.0);
    }

    // Go to angle! Yay!
    public void goToAngle(double angle) {
        
    }

    public void stopPID() {
        pid.cancel();
    }

    public boolean isPIDActive() {
        return pid.active();
    }

    public double getAngle() {
        if(encoder.getPosition() < 0) {
            return 0.0;
        }
        else if(encoder.getPosition() > 90) {
            return 90.0;
        }
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        //will this work if getAngle returns degrees? I do not know
        pid.setMeasurement(getAngle());
        pid.update();

        if (bottomLimitSwitch.get()) {
            stop();
            pid.cancel();
            encoder.setPosition(SubsystemConstants.ENCODER_MIN_ANGLE);
        }
        else if (topLimitSwitch.get()) {
            stop();
            pid.cancel();
            encoder.setPosition(SubsystemConstants.ENCODER_MAX_ANGLE);
        }
        
        motor.set(pid.getSpeed() + SubsystemConstants.FF_VOLTAGE);
    }
}
