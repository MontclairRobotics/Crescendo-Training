package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.Constants.*;
import frc.robot.util.Tunable;

import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static frc.robot.Constants.ArmConstants.*;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax leftMotor = new CANSparkMax(Ports.LEFT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    public Tunable<Double> sprocketKP = Tunable.of(30, "Sprocket KP");
    // private ArmFeedforward angleFeedForward;
    

    public final SparkPIDController leftController = leftMotor.getPIDController();
    public final SparkPIDController rightController = rightMotor.getPIDController();
    public Sprocket() {
        leftController.setP(sprocketKP.get(), 1);
        leftController.setI(0, 1);
        leftController.setD(0, 1);

        rightController.setP(sprocketKP.get(), 1);
        rightController.setI(0, 1);
        rightController.setD(0, 1);

        // leftEncoder = leftMotor.getEncoder();
        // leftEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        // //leftEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        // leftEncoder.setPosition(0);

        // rightEncoder = rightMotor.getEncoder();
        // rightEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        // //rightEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        // rightEncoder.setPosition(0);
        // leftEncoder = leftMotor.getEncoder();
        // rightEncoder = rightMotor.getEncoder();
        // leftEncoder.setPosition(0);
        // rightEncoder.setPosition(0);
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);

        sprocketKP.whenUpdate((p) -> {
            leftController.setP(p, 1);
            rightController.setP(p, 1);
        });
    }

    public void goToAngle(double targetAngle) { // actually goes to rotation. Check RobotContainer for more details.
        leftController.setReference(targetAngle, ControlType.kPosition);
        rightController.setReference(targetAngle, ControlType.kPosition);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public double getEncoderPosition() { // yeah it should be averaged but this was just for testing. 
        return leftMotor.getEncoder().getPosition();
    }
}

