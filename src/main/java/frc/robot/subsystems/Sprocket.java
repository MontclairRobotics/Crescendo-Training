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
    public PIDController pidController = new PIDController(0.125, 0, 0);
    public ArmFeedforward sprocketFeedforward = new ArmFeedforward(0, 0.1,8.91,0.01);

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = leftMotor.getEncoder();

    
 
    public Sprocket() { 
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

    }

        public double getEncoderPosition() { // in rotations
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public void setPosition(double target) { // rotations
        
        pidController.setSetpoint(target);
        
    }
    @Override
    public void periodic() {
        double pidVoltage = pidController.calculate(getEncoderPosition());
        double ffVoltage = sprocketFeedforward.calculate(pidController.getSetpoint() * (Math.PI * 2), 0);
        double voltageOut = pidVoltage + ffVoltage;

        if (voltageOut > 12) {
            voltageOut = 12;
        }

        leftMotor.setVoltage(voltageOut);
        rightMotor.setVoltage(voltageOut);
    }

    
}