// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.MutableMeasure.mutable;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {

   //Creates the motors involved in the shooter mechanism
  public static CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_TOP_MOTOR, MotorType.kBrushless);
  public static CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_BOTTOM_MOTOR, MotorType.kBrushless);
 
  //Creates velocity PID controllers for shooter
   SparkPIDController topController = topMotor.getPIDController();
   SparkPIDController bottomController = bottomMotor.getPIDController();
 
  //Creates the encoders for shooter
  static RelativeEncoder topEncoder = topMotor.getEncoder();
  static RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  
  
  //Create 
  //TODO: Tune feedforward
  public static SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.1639,0.13481,0.025138);
  public static SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.20548,0.13256,0.020699);


  /** Creates a new Shooter. */
  public Shooter() {
    //TODO: Tune PID
    topController.setP(6.6337E-06, 1);
    topController.setD(0, 1);

    bottomController.setP(2.8688E-05, 1);
    bottomController.setD(0, 1);

    topEncoder.setVelocityConversionFactor(1/60.0);
    bottomEncoder.setVelocityConversionFactor(1/60.0);

    Shuffleboard.getTab("Debug").addDouble("Top velocity", velocitySupplier(topMotor));
    Shuffleboard.getTab("Debug").addDouble("Bottom velocity", velocitySupplier(bottomMotor));
  }

  public DoubleSupplier velocitySupplier(CANSparkMax motor){
    return () -> getVelocity(motor);
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
    //sets the top and bottom motors to different velocities
    //method overloading
    public void setVelocity (double topVelocity, double bottomVelocity) {
        double topFeedforwardValue = topFeedforward.calculate(topVelocity);
        double bottomFeedforwardValue = bottomFeedforward.calculate(bottomVelocity);

        topController.setReference(topVelocity, ControlType.kVelocity, 1, topFeedforwardValue );
        bottomController.setReference(bottomVelocity, ControlType.kVelocity, 1, bottomFeedforwardValue );
    }
    //TODO: write this properly
    public boolean isAtVelocity (double velocityRPM) {
      if(getVelocity(topMotor) > (velocityRPM - 1.4) && 
         getVelocity(topMotor) < (velocityRPM + 1.4) &&
         getVelocity(bottomMotor) > (velocityRPM - 1.4) && 
         getVelocity(bottomMotor) < (velocityRPM + 1.4)
      ) {
        return true;
      }
      else return false;
    }

  //Shooter Commands
  public void shootSpeaker(){
    setVelocity(ShooterConstants.SPEAKER_SPEED_RPM);
    if(isAtVelocity(ShooterConstants.SPEAKER_SPEED_RPM)){
      Transport.start();
    }
    System.out.println("hiiiii");
  }

  public static double getVelocity(CANSparkMax motor){
      return motor.getEncoder().getVelocity();
  }

  //TODO: create constants for amp speeds and speaker speeds.
  //TODO: test and get real amp speeds.
  public Command shootSpeakerCommand () {
    return Commands.run(() -> {RobotContainer.shooter.shootSpeaker();});
  
  }
  
  public Command shootAmpCommand () {
    return Commands.runOnce(() -> {RobotContainer.shooter.setVelocity(2000,1500);});
  }

  public void stop(){
    topMotor.set(0);
    bottomMotor.set(0);
    Transport.stop();
  }

  public Command stopCommand(){
    return Commands.runOnce(() -> stop());
  }

  //Start of SysID
  public SysIdRoutine getSysIdRoutine () {

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  // the <> means it is of type whatever is in the things
  MutableMeasure<Voltage> m_appliedVoltage = mutable(Units.Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  MutableMeasure<Angle> m_angle = mutable(Units.Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  MutableMeasure<Velocity<Angle>> m_velocity = mutable((Units.RotationsPerSecond.of(0)));
  // Create a new SysId routine for characterizing the shooter.
  
  final SysIdRoutine theSysRoutine = new SysIdRoutine(

          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),

          //where all the stuff is actually declared. How to set the motors, how to log them, etc.
          new SysIdRoutine.Mechanism(

              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                topMotor.setVoltage(volts.in(Units.Volts));
                bottomMotor.setVoltage(volts.in(Units.Volts));
              },

              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized
              log -> {

                // Records the data for the top shooter motor.
                log.motor("topMotor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            topMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Units.Volts))
                    .angularPosition(m_angle.mut_replace(topEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(topEncoder.getVelocity() / 60, Units.RotationsPerSecond));

                //Record a frame for the bottom shooter motor.
                log.motor("bottomMotor")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            bottomMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Units.Volts))
                    .angularPosition(m_angle.mut_replace(bottomEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(bottomEncoder.getVelocity() / 60, Units.RotationsPerSecond));
              },

              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")

              this));

  //returns what we just created
  return theSysRoutine;

  }
  /** 
   * Returns a command that will execute a quasistatic test in the given direaction
   * 
   * @param direction needs to be SysIdRoutine.Direction.kReverse or SysIdRoutine.Direction.kForward
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().quasistatic(direction);
  }
  /**
   * Returns a command that will execute a dynamic test in the given direction.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return getSysIdRoutine().dynamic(direction);
  }
  

  //This is the command group that runs the SysIDRoutine
  public SequentialCommandGroup SysIDCommand = new SequentialCommandGroup(
      sysIdDynamic(SysIdRoutine.Direction.kForward),
      Commands.waitSeconds(5),
      sysIdDynamic(SysIdRoutine.Direction.kReverse),
      Commands.waitSeconds(5),
      sysIdQuasistatic(SysIdRoutine.Direction.kForward),
      Commands.waitSeconds(5),
      sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
  );

  //END OF SYSID

  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
