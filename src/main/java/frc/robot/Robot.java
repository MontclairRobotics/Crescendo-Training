// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.Shooter;
import frc.robot.vision.Limelight;


import org.littletonrobotics.junction.LoggedRobot;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    /* LOGGING */
    SmartDashboard.putNumber("topMotor Velocity RPM", Shooter.getVelocityRPM(Shooter.topMotor));
    SmartDashboard.putNumber("bottomMotor Velocity RPM", Shooter.getVelocityRPM(Shooter.bottomMotor));
    Shuffleboard.getTab("Debug").addDouble("Sprocket angle", RobotContainer.sprocket.getRawPositionSupplier());
    Shuffleboard.getTab("Debug").addDouble("TX", Limelight.txSupplier()).withSize(2,1);
    //Shuffleboard.getTab("Debug").addDouble("setPoint", RobotContainer.drivetrain.setPointSupplier()).withSize(2,1);
    //Shuffleboard.getTab("Debug").addDouble("response", RobotContainer.drivetrain.responseSupplier()).withSize(2,1);
    //Shuffleboard.getTab("Debug").addBoolean("Is Robot At Set Point", RobotContainer.drivetrain.isRobotAtAngleSetPoint()).withSize(2,1);
    Shuffleboard.getTab("Debug").addDouble("Odometry Heading", RobotContainer.drivetrain.odometryHeadingDoubleSupplier()).withSize(2,1);
    //Shuffleboard.getTab("Debug").addDouble("wrapped setpoint", RobotContainer.drivetrain.wrappedSetPointSupplier()).withSize(2,1);
    //Shuffleboard.getTab("Debug").addDouble("ty", Limelight.tySupplier()).withSize(2,1);
    Shuffleboard.getTab("Debug").addDouble("best fit angle", RobotContainer.limelight.bestFitAngleSupplier()).withSize(2,1);
    Shuffleboard.getTab("Debug").addBoolean("Sprocket at angle?", RobotContainer.sprocket.isAtAngleSupplier());
    Shuffleboard.getTab("Debug").addBoolean("Is robot aligned tx",  () -> RobotContainer.drivetrain.isAlignedTX());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //System.out.println("feedback: " + AutoCommands.auto.feedback);
    //System.out.println(AutoCommands.auto.isAutoStringValid);
    //System.out.println(AutoCommands.auto.autoString);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.println("Sprocket: "+ RobotContainer.sprocket.getRawPositionSupplier());
    // System.out.println("From:" + RobotContainer.auto.isFromScoringLocation);
    // System.out.println("Going to:" + RobotContainer.auto.isGoingToScoringLocation);
    // System.out.println(RobotContainer.auto.isIn(Auto.autoString.charAt(0), RobotContainer.auto.scoringLocations));
    //System.out.println(RobotContainer.intake.beambreak.get());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
    
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
