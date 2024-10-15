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

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
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
  //Creates feedforward

  //old constants
 // public static SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.1639,0.13481,0.025138);
 // public static SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.20548,0.13256,0.020699);
 //new constants
  public static SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(ShooterConstants.TOP_SHOOTER_FF_KS, ShooterConstants.TOP_SHOOTER_FF_KV, ShooterConstants.TOP_SHOOTER_FF_KA);
  public static SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(ShooterConstants.BOTTOM_SHOOTER_FF_KS, ShooterConstants.BOTTOM_SHOOTER_FF_KV, ShooterConstants.BOTTOM_SHOOTER_FF_KA);
  /** Creates a new Shooter. */
  public Shooter() {
    //old constants
    // topController.setP(6.6337E-06*60, 1);
    // topController.setD(0, 1);

    // bottomController.setP(2.8688E-05, 1);
    // bottomController.setD(0, 1);

    //new constants
    topController.setP(ShooterConstants.TOP_SHOOTER_PID_KP, 1);
    topController.setD(ShooterConstants.TOP_SHOOTER_PID_KD, 1);

    bottomController.setP(ShooterConstants.BOTTOM_SHOOTER_PID_KP, 1);
    bottomController.setD(ShooterConstants.BOTTOM_SHOOTER_PID_KD, 1);

  //   topEncoder.setVelocityConversionFactor();
  //  bottomEncoder.setVelocityConversionFactor(1/60.0);

    // Shuffleboard.getTab("Debug").addDouble("Top velocity RPM", velocitySupplierRPM(topMotor));
    // Shuffleboard.getTab("Debug").addDouble("Bottom velocity RPM", velocitySupplierRPM(bottomMotor));

    // Shuffleboard.getTab("Debug").addDouble("Top velocity RPSSS", velocitySupplierRPS(topMotor));
    // Shuffleboard.getTab("Debug").addDouble("Bottom velocity RPSSS", velocitySupplierRPS(bottomMotor));
  }
  //supplier of Velocity RPMMMMM
  public DoubleSupplier velocitySupplierRPM(CANSparkMax motor){
    return () -> getVelocityRPM(motor);
  }
  //supplier of velocity RPSSSSS
  public DoubleSupplier velocitySupplierRPS(CANSparkMax motor){
    return () -> getVelocityRPS(motor);
  }
    //sets both shooter motors to the same velocity, meaning the motors will spin to that velocity
    public void setVelocity (double velocity) {
        double topFeedforwardValue = topFeedforward.calculate(velocity);
        double bottomFeedforwardValue = bottomFeedforward.calculate(velocity);
        topController.setReference(velocity, ControlType.kVelocity, 1, topFeedforwardValue );
        bottomController.setReference(velocity, ControlType.kVelocity, 1, bottomFeedforwardValue );
    }
    public void setVelocity (double topVelocity, double bottomVelocity) {
        double topFeedforwardValue = topFeedforward.calculate(topVelocity);
        double bottomFeedforwardValue = bottomFeedforward.calculate(bottomVelocity);
        topController.setReference(topVelocity, ControlType.kVelocity, 1, topFeedforwardValue );
        bottomController.setReference(bottomVelocity, ControlType.kVelocity, 1, bottomFeedforwardValue );
    }
    //returns true if input velocity (RPM) is within the deadband (RPM)
    public boolean isAtVelocityRPS (double topSetpointRPS, double bottomSetpointRPS) {
     if(Math.abs(getVelocityRPS(topMotor) - topSetpointRPS) < ShooterConstants.SHOOTER_VELOCITY_DEADBAND_RPS
       && Math.abs(getVelocityRPS(bottomMotor) - bottomSetpointRPS) < ShooterConstants.SHOOTER_VELOCITY_DEADBAND_RPS) return true;
      else return false;
    }
    //returns true if input velocity RPS is within the deadband RPS
    public boolean isAtVelocityRPM( double topSetpoint, double bottomSetpoint){
       if(Math.abs(getVelocityRPM(topMotor) - topSetpoint) < ShooterConstants.SHOOTER_VELOCITY_DEADBAND_RPM
       && Math.abs(getVelocityRPM(bottomMotor) - bottomSetpoint) < ShooterConstants.SHOOTER_VELOCITY_DEADBAND_RPM) return true;
      else return false;
    }

  //spins the wheels to shoot at DIFFERENT speeds, when they reach said speed, transport will start.
  public void shootRPM(double topVelocityRPM, double bottomVelocityRPM) {
    setVelocity(topVelocityRPM, bottomVelocityRPM);
    if(isAtVelocityRPM(topVelocityRPM, bottomVelocityRPM)) Transport.reverse();
  }

  //spins the wheels to shoot at the SAME speed, when they reach said speed, transport will start.
  public void shootRPM(double velocityRPM) {
    setVelocity(velocityRPM);
    if(isAtVelocityRPM(velocityRPM, velocityRPM)) Transport.start();
  }

  //gets the velocity given a certain motor in RPM
  public static double getVelocityRPM(CANSparkMax motor){
      return motor.getEncoder().getVelocity();
  }

  //gets the velocity of input motor in RPS
  public static double getVelocityRPS(CANSparkMax motor){
    return motor.getEncoder().getVelocity()/60;
  }
  public void intakeSource(){
    RobotContainer.shooter.shootRPM(-2000);
    if(isAtVelocityRPM(-2000, -2000)){
      Transport.start();
    };
  }
  public void intakeSource2(){
    if(!RobotContainer.intake.beambreak.get()) {
      RobotContainer.shooter.stop();
      Transport.stop();
    } else {
    RobotContainer.shooter.shootRPM(-2000);
    if(isAtVelocityRPM(-2000, -2000)){
      Transport.start();
    };
    }
  }
  public Command intakeSourceCommand(){
    return Commands.sequence(
      Commands.runOnce(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(52)), RobotContainer.sprocket),
      Commands.run(() -> {intakeSource();})
      .onlyWhile(RobotContainer.intake.getBB()),
      Commands.run(()-> {intakeSource2();})
      .onlyWhile(RobotContainer.intake.getReverseBB())
      );
    
  }
  public void stopIntakeSource(){
    RobotContainer.shooter.stop();
    Transport.stop();
  }
  public Command stopIntakeSourceCommand(){
    return Commands.runOnce(() -> stopIntakeSource());
  }
  public Command scoreAmp(){
    return Commands.sequence(
      Commands.runOnce(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(ShooterConstants.AMP_SCORE_ANGLE)), this),
      Commands.waitSeconds(.6),
      shootAmpCommand());
  }
  public Command shootSpeakerCommand () {
    return Commands.run(() -> {RobotContainer.shooter.shootRPM(ShooterConstants.SHOOT_SPEAKER_VELOCITY);});
  }
  public Command shootAmpCommand () {
    return Commands.run(() -> {RobotContainer.shooter.shootRPM(ShooterConstants.AMP_TOP_SPEED, ShooterConstants.AMP_BOTTOM_SPEED);});
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
