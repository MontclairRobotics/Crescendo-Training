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

    public Tunable<Double> sprocketKP = Tunable.of(3, "Sprocket KP");
    
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = leftMotor.getEncoder();
    public Sprocket() {

        leftMotor.getPIDController().setP(sprocketKP.get(), 0);
    

        rightMotor.getPIDController().setP(sprocketKP.get(),0);
        


        leftMotor.setInverted(true);
        rightMotor.setInverted(false);


        
        sprocketKP.whenUpdate((p)-> {
            leftMotor.getPIDController().setP(sprocketKP.get(), 0);
            rightMotor.getPIDController().setP(sprocketKP.get(),0);
        });
        
    }

    public double getEncoderPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }
    
    public void goToRotations(double target) {
        leftMotor.getPIDController().setReference(target, CANSparkBase.ControlType.kPosition);
        rightMotor.getPIDController().setReference(target, CANSparkBase.ControlType.kPosition);
    }

    
}