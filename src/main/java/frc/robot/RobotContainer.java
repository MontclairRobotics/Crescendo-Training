// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {

    public static Shooter shooter = new Shooter();
    CommandPS5Controller driverController = new CommandPS5Controller(0);
    CommandPS5Controller operatorController = new CommandPS5Controller(1);
    CommandPS5Controller testingController = new CommandPS5Controller(2);

  public RobotContainer() {

    

    configureBindings();

  }

  //Configure key bindings
  private void configureBindings() {
    
    testingController.cross().onTrue(Shooter.shootSpeaker());
    testingController.square().onTrue(Shooter.shootAmp());
    testingController.circle().onTrue(RobotContainer.shooter.SysIDCommand);
    
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
