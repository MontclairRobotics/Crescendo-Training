// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
import swervelib.SwerveDrive;


public class RobotContainer {
    //Creates an intake object. Must be referenced in other classes by doing:
    //RobotContainer.intake.insertExampleMethod();
    public static Intake intake = new Intake();

    //Creates new Sprocket object. If you get static issues try referencing the object and then
    //the method.
    public static Sprocket sprocket = new Sprocket();

    public static Shooter shooter = new Shooter();
    public static Drivetrain drivetrain = new Drivetrain();

    //new auto object
    public static Auto auto = new Auto();

    //new autoBuilder object
    public static AutoBuilder autoBuilder = new AutoBuilder();

    //new Shooter object


    //all the different controllers used to configure button bindings
    public static CommandPS5Controller driverController = new CommandPS5Controller(0);
    public static CommandPS5Controller operatorController = new CommandPS5Controller(1);
    public static CommandPS5Controller testingController = new CommandPS5Controller(2);

    //Constructor...these comments are killing me...
    public RobotContainer() {

    //sets the default command for the drivetrain to take the joystick inputs from
    //the driverController and converts them into velocitys for the robot to drive
    //in teleoperated mode
    drivetrain.setDefaultCommand(drivetrain.driveCommand());

    //sets the default command for the sprocket to be in manual control
    sprocket.setDefaultCommand(sprocket.sprocketDefaultCommand());

    //calls the configureBindings method which binds the buttons to certain commands
    configureBindings();

  }//end of constructor.

  //Configure key bindings
  private void configureBindings() {
    
    //Operator bindings
    operatorController.cross().whileTrue(shooter.shootSpeakerCommand()).onFalse(shooter.stopCommand());
    operatorController.square().onTrue(shooter.shootAmpCommand()).onFalse(shooter.stopCommand());

    //intakes
    operatorController.L1().whileTrue(intake.inhaleCommand());
    operatorController.R1().whileTrue(intake.exhaleCommand());

    //sprocket temporary setAngleCommand
    operatorController.circle().onTrue(sprocket.setAngleCommand(35));
    operatorController.triangle().onTrue(sprocket.setAngleCommand(50));
    //brake mode
    operatorController.touchpad().onTrue(sprocket.setCoastModeCommand().ignoringDisable(true)).onFalse(sprocket.setBrakeModeCommand().ignoringDisable(true));
    // ControllerTools.getDPad(DPad.DOWN, operatorController).onTrue(sprocket.downCommand()).onFalse(sprocket.stopCommand());
    // ControllerTools.getDPad(DPad.UP, operatorController).onTrue(sprocket.upCommand()).onFalse(sprocket.stopCommand());
    // //driver bindings
    driverController.L1().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());
    driverController.R2().onTrue(drivetrain.zeroGyro());

    // ControllerTools.getDPad(DPad.UP, driverController).onTrue(RobotContainer.sprocket.setAngleCommand(63));
    // ControllerTools.getDPad(DPad.DOWN, driverController).onTrue(RobotContainer.sprocket.setAngleCommand(26));
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
