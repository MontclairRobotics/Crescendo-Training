// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Ports;


public class Shooter extends SubsystemBase {
  
   //Creates the motors involved in the shooter mechanism
  public static CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_TOP_MOTOR, MotorType.kBrushless);
  public static CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_BOTTOM_MOTOR, MotorType.kBrushless);
  public static CANSparkMax transport = new CANSparkMax(Ports.SHOOTER_MOTOR_TRANSPORT, MotorType.kBrushless);
 
  //Creates velocity PID controllers for shooter
  static SparkPIDController topController = topMotor.getPIDController();
  static SparkPIDController bottomController = bottomMotor.getPIDController();
 
  //Creates the encoders for shooter
  static RelativeEncoder topEncoder = topMotor.getEncoder();
  RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
   
  //Create 
  //TODO: Tune feedforward
  public static SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0,0,0);
  public static SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0,0,0);

  //Booleans
  private static boolean isAtVelocity = false;

  /** Creates a new Shooter. */
  public Shooter() {
    //TODO: Tune PID
    topController.setP(4, 1);
    topController.setD(0, 1);

    bottomController.setP(4, 1);
    bottomController.setD(0, 1);
  }

    //Shooter methods

    //sets both shooter motors to the same velocity, meaning the motors will spin to that velocity
    public void setVelocity (double velocity) {
        //calculates the feedforward value using the .calculate method and sets it to 
        //a double to be used in the .setReference, which will automaitcally spin the motors
        double topFeedforwardValue = topFeedforward.calculate(velocity);
        double bottomFeedforwardValue = bottomFeedforward.calculate(velocity);

        //sets the reference using the target velocity, the control type, the pidslot, and the calculated
        //feedforward value above
        topController.setReference(velocity, ControlType.kVelocity, 1, topFeedforwardValue );
        bottomController.setReference(velocity, ControlType.kVelocity, 1, bottomFeedforwardValue );
    }
    public void setVelocity (double topVelocity, double bottomVelocity) {
        double topFeedforwardValue = topFeedforward.calculate(topVelocity);
        double bottomFeedforwardValue = bottomFeedforward.calculate(bottomVelocity);

        topController.setReference(topVelocity, ControlType.kVelocity, 1, topFeedforwardValue );
        bottomController.setReference(bottomVelocity, ControlType.kVelocity, 1, bottomFeedforwardValue );
    }
    //TODO: write this properly
    public void isAtVelocity (double velocity) {
    }

  //Shooter Commands
  public static double getVelocity(CANSparkMax motor){
      return motor.getEncoder().getVelocity();
  }
  public static Command shootSpeaker () {
    return Commands.run(() -> {RobotContainer.shooter.setVelocity(4000);});
  }

  public static Command shootAmp () {
    return Commands.run(() -> {RobotContainer.shooter.setVelocity(2000,1500);});
  }

  public SysIdRoutine getSysIdRoutine () {

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  MutableMeasure<Voltage> m_appliedVoltage = mutable(Units.Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  MutableMeasure<Angle> m_angle = mutable(Units.Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  MutableMeasure<Velocity<Angle>> m_velocity = mutable(Units.RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  
  final SysIdRoutine theSysRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                topMotor.setVoltage(volts.in(Units.Volts));
                bottomMotor.setVoltage(volts.in(Units.Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("topMotor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            topMotor.get() * RobotController.getBatteryVoltage(), Units.Volts))
                    .angularPosition(m_angle.mut_replace(topEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(topEncoder.getVelocity() / 60, Units.RotationsPerSecond));

                        log.motor("bottomMotor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            bottomMotor.get() * RobotController.getBatteryVoltage(), Units.Volts))
                    .angularPosition(m_angle.mut_replace(bottomEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(bottomEncoder.getVelocity() / 60, Units.RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter"
              this));

  return theSysRoutine;

  }
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().quasistatic(direction);
  }
  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().dynamic(direction);
  }
  static SequentialCommandGroup SysIDCommand = new SequentialCommandGroup(
      sysIdDynamic(forward),
      sysIdDynamic(backward),
      sysIdDynamic(forward),
      sysIdDynamic(backward),

      sysIdQuasistatic(forward),
      sysIdQuasistatic(backward),
      sysIdQuasistatic(forward),
      sysIdQuasistatic(backward)

  );

  public void periodic() {
    // This method will be called once per scheduler run
   
    

  
  }
}
