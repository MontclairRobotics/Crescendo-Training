// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    public static Intake intake = new Intake();
    public static Shooter shooter = new Shooter();
    CommandPS5Controller driverController = new CommandPS5Controller(0);
    CommandPS5Controller operatorController = new CommandPS5Controller(1);
    CommandPS5Controller testingController = new CommandPS5Controller(2);

  public RobotContainer() {

    

    configureBindings();

  }

  //Configure key bindings
  private void configureBindings() {
    
    operatorController.cross().onTrue(Shooter.shootSpeaker());
    operatorController.square().onTrue(Shooter.shootAmp());
    operatorController.circle().onTrue(RobotContainer.shooter.SysIDCommand);
    operatorController.L1().onTrue(RobotContainer.intake.inhaleCommand());
    operatorController.R1().onTrue(RobotContainer.intake.exhaleCommand());
    
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
