package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.Constants.*;
import frc.robot.util.Tunable;

import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static frc.robot.Constants.ArmConstants.*;

public class Sprocket extends SubsystemBase {
    
    private final CANSparkMax leftMotor = new CANSparkMax(Ports.LEFT_ANGLE_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Ports.RIGHT_ANGLE_MOTOR_PORT, MotorType.kBrushless);

    // public Tunable<Double> sprocketKP = Tunable.of(3, "Sprocket KP");
    public PIDController pidController = new PIDController(0.8, 0, 0);
    public ArmFeedforward sprocketFeedforward = new ArmFeedforward(0.125, 0.1,8.91,0.01);

    // private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    // private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    public DutyCycleEncoder absEncoder = new DutyCycleEncoder(1);

    public LimitSwitch bottomLimitSwitch = new LimitSwitch(0, true);
    public LimitSwitch topLimitSwitch = new LimitSwitch(1, true);
    public Sprocket() { 

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        
        absEncoder.setDistancePerRotation(360.0);
    }

    public double getEncoderPosition() { 
        return -absEncoder.getDistance() + 100; // TODO: correct offset please
    }

    public void setTargetAngle(double target) {
        pidController.setSetpoint(target);
    }
    public void stop() {
      leftMotor.set(0);
      rightMotor.set(0);
    }
    @Override
    public void periodic() {

        double radianSetpoint = pidController.getSetpoint() * (Math.PI / 180.0); 

        double pidVoltage = pidController.calculate(getEncoderPosition());
        double ffVoltage = sprocketFeedforward.calculate(radianSetpoint, 0);

        double voltageOut = pidVoltage + ffVoltage;

        if (voltageOut > 12) { // amazing code right here.
            voltageOut = 12;
            System.out.println("Combined voltage was greater than 12");
        }
        if ((bottomLimitSwitch.get() || topLimitSwitch.get()) || (getEncoderPosition() > (ENCODER_MAX_ANGLE-2) || getEncoderPosition() < (ENCODER_MIN_ANGLE + 2))) {
          System.out.println("A limit switch was hit!");
          stop();
        } else {
          leftMotor.setVoltage(voltageOut);
          rightMotor.setVoltage(voltageOut);
        }
    }

    
}