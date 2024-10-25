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
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.IntakeCommands;
import frc.robot.Commands.ShooterCommands;
import frc.robot.Commands.SprocketCommands;
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

    //INSTANTIATING ALL OF THE COMMAND CLASSES
    public static IntakeCommands intakecommands = new IntakeCommands();
    public static ShooterCommands shootercommands = new ShooterCommands();
    public static SprocketCommands sprocketcommands = new SprocketCommands();
    public static DriveCommands drivecommands = new DriveCommands();
    public static AutoCommands autocommands = new AutoCommands();

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
    /*
     * 
     * DEFAULT COMMANDS
     * 
     */

    //DRIVING JOYSTICK INPUTS
    drivetrain.setDefaultCommand(drivecommands.driveCommand());

    //SPROCKET MANUAL CONTROL
    sprocket.setDefaultCommand(sprocketcommands.sprocketDefaultCommand());

    //CLIMBER JOYSTICK INPUT
    climbers.setDefaultCommand(climbers.climbersDefaultCommand());

    /* 
     * 
     * OPERATOR BINDINGS
     * 
    */
    
    //SCORES SPEAKER (SUBWOOFER OR SCORING MODE)
    operatorController.circle()
      .whileTrue(shootercommands.scoreSpeaker(false))
      .onFalse(shootercommands.stop());
    
    //SCORES AMP
    operatorController.triangle()
      .whileTrue(shootercommands.scoreAmp())
      .onFalse(shootercommands.stop());
    
    //RUNS THE INTAKE
    operatorController.L2()
      .onTrue(intakecommands.intake())
      .onFalse(intakecommands.stopIntake());

    //RUNS THE OUTTAKE
    operatorController.R2()
      .onTrue(intakecommands.outtake())
      .onFalse(intakecommands.stopIntake());

    //SO SPROCKET CAN BE MOVED WHEN DISABLED
    operatorController.touchpad()
      .onTrue(sprocketcommands.setCoastModeCommand().ignoringDisable(true))
      .whileFalse(sprocketcommands.setBrakeModeCommand().ignoringDisable(true));

    //INTAKES FROM THE SHOOTER
    operatorController.R1()
      .whileTrue(intakecommands.intakeSourceCommand())
      .onFalse(intakecommands.stopIntake());
    
    /*
    *
    *DRIVER BINDINGS
    *
    */

    //ROBOT RELATIVE DRIVING
    driverController.L2()
      .onTrue(drivecommands.toRobotRelativeCommand())
      .onFalse(drivecommands.toFieldRelativeCommand());

    //ZEROS THE GYRO
    driverController.touchpad()
      .onTrue(drivecommands.zeroGyro());

    //SCORING MODE
    driverController.R2().and(()->LimelightHelpers.getTV(Limelight.llname))
      .whileTrue(drivecommands.scoringMode(false, false))
      .onFalse(drivecommands.stopScoringModeCommand());

    //ALIGN FIELD RELATIVE TO 90 DEGREES (INTAKE POINT TO THE LEFT)
    driverController.square()
      .onTrue(drivecommands.alignToAngleFieldRelative(90, false));

    //ALIGN FIELD RELATIVE TO 180 DEGREES (INTAKE POINT TOWARDS YOU)
    driverController.cross()
     .onTrue(drivecommands.alignToAngleFieldRelative(180, false));

    //ALIGN FIELD RELATIVE -90 DEGREES (INTAKE POINTING RIGHT)
    driverController.circle()
      .onTrue(drivecommands.alignToAngleFieldRelative(-90, false));
    
    //ALIGN FIELD RELATIVE 0 DEGREES (INTAKE POINTING AWAY FROM YOU)
    driverController.triangle()
      .onTrue(drivecommands.alignToAngleFieldRelative(0, false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    auto.autoSequencer();
    return Commands.runOnce(() -> autocommands.runAutoSequentialCommandGroup());
  }
}
