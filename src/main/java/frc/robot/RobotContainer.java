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
import frc.robot.Constants.IntakeConstants;
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
import frc.robot.vision.LimelightHelpers;


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
    LimelightHelpers.setCameraMode_Driver(Limelight.llname);
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
    //TESTING BINDINGS

    /*OPERATOR BINDINGS*/
    
    //shoots speaker without angle changing
    operatorController.circle().whileTrue(shooter.scoreSpeakerCommand()).onFalse(shooter.stopCommand());
    //shoots amp hopefully works
    operatorController.triangle().whileTrue(shooter.scoreAmp()).onFalse(shooter.stopCommand());
    //intake because inverts
    operatorController.L2().onTrue(intake.intakeCommand()).onFalse(intake.stopCommand());
    //outtake because inverts
    operatorController.R2().onTrue(intake.outtakeCommand()).onFalse(intake.stopCommand());
    //jank fix
    //makes sprocket able to be moved when disabled
    operatorController.touchpad().onTrue(sprocket.setCoastModeCommand().ignoringDisable(true)).whileFalse(sprocket.setBrakeModeCommand().ignoringDisable(true));
    //intake from source. UNTESTED
    operatorController.R1().whileTrue(shooter.intakeSourceCommand()).onFalse(shooter.stopIntakeSourceCommand());
    

    /*DRIVER BINDINGS*/
    //robot relative driving
    driverController.L2().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());
    //zero gyro
    driverController.touchpad().onTrue(drivetrain.zeroGyro());
    //scoring mode. DOES NOT WORK YET
    driverController.R2().onTrue(drivetrain.scoringMode(false)).onFalse(drivetrain.stopScoringModeCommand());
    //align to angle for testing. DOES NOT WORK
    driverController.R2().onTrue(drivetrain.alignRobotRelativeCommand(30, false));
    driverController.square().onTrue(drivetrain.alignFieldRelativeCommand(90, false));
    driverController.cross().onTrue(drivetrain.alignFieldRelativeCommand(180, false));
    driverController.circle().onTrue(drivetrain.alignFieldRelativeCommand(-90, false));
    driverController.triangle().onTrue(drivetrain.alignFieldRelativeCommand(0, false));

    
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
