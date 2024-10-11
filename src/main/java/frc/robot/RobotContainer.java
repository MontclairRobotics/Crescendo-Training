// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumSet;

import javax.swing.TransferHandler.TransferSupport;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.subsystems.Transport;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
import frc.robot.vision.Limelight;


public class RobotContainer {
  
    //INSTANTIATING ALL OF THE SUBSYSTEMS
    public static Intake intake = new Intake();
    public static Sprocket sprocket = new Sprocket();
    public static Shooter shooter = new Shooter();
    public static Drivetrain drivetrain = new Drivetrain();
    public static Climbers climbers = new Climbers();
    public static Auto auto = new Auto();
    public static AutoBuilder autoBuilder = new AutoBuilder();
    public static Limelight limelight = new Limelight();
    public static Transport transport = new Transport();

    //INSTANTIATING THE CONTROLLERS
    public static CommandPS5Controller driverController = new CommandPS5Controller(0);
    public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
    public static CommandPS5Controller testingController = new CommandPS5Controller(2);
  
    //the measely constructor
    public RobotContainer() {
    //configures bindings
    configureBindings();
    }

  private void configureBindings() {
    //DEFAULT COMMANDS

    //driving duh
    drivetrain.setDefaultCommand(drivetrain.driveCommand());
    //sprocket manual control
    sprocket.setDefaultCommand(sprocket.sprocketDefaultCommand());
    //climber default commmand
    climbers.setDefaultCommand(climbers.climbersDefaultCommand());
    
    //OPERATOR BINDINGS
    
    //shoots speaker without angle changing
    operatorController.circle().whileTrue(shooter.shootSpeakerCommand()).onFalse(shooter.stopCommand());
    //shoots amp hopefully works
    operatorController.triangle().whileTrue(shooter.scoreAmp()).onFalse(shooter.stopCommand());
    //intake
    operatorController.L2().onTrue(intake.intakeCommand()).onFalse(intake.stopCommand());
    //outtake
    operatorController.R2().onTrue(intake.outtakeCommand()).onFalse(intake.stopCommand());
    //sprocket angle commands for fun
    operatorController.cross().onTrue(sprocket.setAngleCommand(35));
    operatorController.square().onTrue(sprocket.setAngleCommand(50));
    //makes sprocket able to be moved when disabled
    operatorController.touchpad().onTrue(sprocket.setCoastModeCommand().ignoringDisable(true)).whileFalse(sprocket.setBrakeModeCommand().ignoringDisable(true));
    //intake from source. UNTESTED
    operatorController.L1().onTrue(shooter.intakeSourceCommand());
    
    /*DRIVER BINDINGS*/
    //robot relative driving
    driverController.L2().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());
    //zero gyro
    driverController.touchpad().onTrue(drivetrain.zeroGyro());
    //scoring mode. DOES NOT WORK YET
    driverController.R2().whileTrue(drivetrain.scoringMode());
    //align to angle for testing. DOES NOT WORK
    driverController.cross().onTrue(drivetrain.alignRobotRelativeCommand(90));
    driverController.circle().onTrue(drivetrain.alignFieldRelativeCommand(90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    auto.autoSequencer();
    return Commands.runOnce(() -> auto.runAutoSequentialCommandGroup());
  }
}
