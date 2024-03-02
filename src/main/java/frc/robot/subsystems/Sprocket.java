package frc.robot.subsystems;


import frc.robot.LimitSwitch;
import frc.robot.RobotContainer;
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
import com.revrobotics.CANSparkBase.IdleMode;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    public PIDController pidController = new PIDController(0.8, 0, 0);
    
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();



    public Tunable<Double> voltageOut = Tunable.of(6.0, "Sprocket Voltage");


    public Sprocket() { 

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE*(1/60));
        leftEncoder.setPosition(ENCODER_MIN_ANGLE);

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE);
        leftEncoder.setVelocityConversionFactor(1/SPROCKET_ROTATIONS_PER_DEGREE*(1/60));
        rightEncoder.setPosition(ENCODER_MIN_ANGLE);
        pidController.setTolerance(2);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        

    }

    public double getEncoderPosition() {
      return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }
    // public double getEncoderPosition() { 
    //     return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    // }

    public void setTargetAngle(double target) {
        pidController.setSetpoint(target);
        
    }
    public void stop() {
      leftMotor.set(0);
      rightMotor.set(0);
    }
    @Override
    public void periodic() {
        ///double voltageOut = pidController.calculate(getEncoderPosition());
        
        // leftMotor.setIdleMode(IdleMode.kBrake);
        // if (pidController.atSetpoint()) {
        //     stop();
        // } else {
        //     leftMotor.setVoltage(voltageOut);
        //     rightMotor.setVoltage(voltageOut);
        // }
        if (RobotContainer.driverController.cross().getAsBoolean()) {
            leftMotor.setVoltage(voltageOut.get());
            rightMotor.setVoltage(voltageOut.get());
        }
        
        
        
        
    }
    

    
}