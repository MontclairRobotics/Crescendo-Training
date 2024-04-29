// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {

    CommandPS4Controller driverController = new CommandPS4Controller(0);
    CommandPS4Controller operatorController = new CommandPS4Controller(1);
    CommandPS4Controller testingController = new CommandPS4Controller(2);

  public RobotContainer() {

   

    configureBindings();

  }

  //Configure key bindings
  private void configureBindings() {
    
    testingController.cross().onTrue(Shooter.speaker());
    testingController.square().onTrue(Shooter.amp());
    
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
