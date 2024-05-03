// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sprocket;
import frc.robot.util.ControllerTools;
import frc.robot.util.ControllerTools.DPad;
public class RobotContainer {
  //pissing james rechs off by doing indents
    public static       Intake intake = new Intake();
    // public       static Sprocket sprocket = new Sprocket();
    public static       Shooter shooter = new Shooter();
    public       static CommandPS5Controller driverController = new CommandPS5Controller(0);
    public static       CommandPS5Controller operatorController = new CommandPS5Controller(1);
    public       static CommandPS5Controller testingController = new CommandPS5Controller(2);
    public Drivetrain drivetrain = new Drivetrain();
  public RobotContainer() {

    drivetrain.setDefaultCommand(drivetrain.drive());
    configureBindings();

  }

  //Configure key bindings
  private void configureBindings() {
    operatorController.cross().onTrue(RobotContainer.shooter.shootSpeaker());
    operatorController.square().onTrue(RobotContainer.shooter.shootAmp());
    operatorController.circle().onTrue(RobotContainer.shooter.SysIDCommand);
    operatorController.L1().whileTrue(RobotContainer.intake.inhaleCommand());
    operatorController.R1().whileTrue(RobotContainer.intake.exhaleCommand());
    driverController.L1().onTrue(drivetrain.toRobotRelativeCommand()).onFalse(drivetrain.toFieldRelativeCommand());
    driverController.R2().onTrue(drivetrain.zeroGyro());
  //   ControllerTools.getDPad(DPad.UP, driverController).onTrue(RobotContainer.sprocket.setAngleCommand(63));
  //   ControllerTools.getDPad(DPad.DOWN, driverController).onTrue(RobotContainer.sprocket.setAngleCommand(26));
  // 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> {});
  }
}
