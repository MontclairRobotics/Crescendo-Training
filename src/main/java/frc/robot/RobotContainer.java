// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
import frc.robot.vision.Limelight;


public class RobotContainer {
    //Creates an intake object. Must be referenced in other classes by doing:
    //RobotContainer.intake.insertExampleMethod();
  
    //INSTANTIATING ALL OF THE SUBSYSTEMS
    public static Intake intake = new Intake();
    public static Sprocket sprocket = new Sprocket();
    public static Shooter shooter = new Shooter();
    public static Drivetrain drivetrain = new Drivetrain();
    public static Climbers climbers = new Climbers();
    public static Auto auto = new Auto();
    public static AutoBuilder autoBuilder = new AutoBuilder();
    public static Limelight limelight = new Limelight();

    //INSTANTIATING THE CONTROLLERS
    public static CommandPS5Controller driverController = new CommandPS5Controller(0);
    public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
    public static CommandPS5Controller testingController = new CommandPS5Controller(2);

    public static boolean shouldShiftClimberDefaultCommand;
    public RobotContainer() {

    //TO CONFIGURE BINDINGS
    configureBindings();

    /*sets the default command for the drivetrain to take the joystick inputs from
    the driverController and converts them into velocities for the robot to drive
    in teleoperated mode*/
    drivetrain.setDefaultCommand(drivetrain.driveCommand());

    //DEFAULT COMMAND FOR SPROCKET IS MANUAL CONTROL
    sprocket.setDefaultCommand(sprocket.sprocketDefaultCommand());

    climbers.setDefaultCommand(climbers.DefaultCommand());
    
  }

  private void configureBindings() {
    
    //Operator bindings
    operatorController.circle().whileTrue(shooter.shootSpeakerCommand()).onFalse(shooter.stopCommand());
    operatorController.triangle().onTrue(shooter.shootAmpCommand()).onFalse(shooter.stopCommand());

    shouldShiftClimberDefaultCommand = operatorController.L1().getAsBoolean();
    operatorController.L2().whileTrue(intake.inhaleCommand());
    operatorController.R2().whileTrue(intake.exhaleCommand());

    //FOR TESTING
    operatorController.cross().onTrue(sprocket.setAngleCommand(35));
    operatorController.square().onTrue(sprocket.setAngleCommand(50));
    operatorController.touchpad().onTrue(sprocket.setCoastModeCommand().ignoringDisable(true)).onFalse(sprocket.setBrakeModeCommand().ignoringDisable(true));
    
    //DRIVER BINDINGS
    driverController.L1().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());
    driverController.touchpad().onTrue(drivetrain.zeroGyro());
    driverController.R2().whileTrue(drivetrain.scoringMode());
    driverController.cross().onTrue(drivetrain.alignRobotRelativeCommand(90));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> auto.runAutoSequentialCommandGroup());
  }
}
