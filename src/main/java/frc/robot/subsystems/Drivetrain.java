// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

  import java.io.File;
  import edu.wpi.first.wpilibj.Filesystem;
  import swervelib.parser.SwerveParser;
  import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  /** Creates a new SwerveDrive. */

  boolean FieldRelative;
  double rotationSpeed;
  double maximumSpeed;
  File swerveJsonDirectory;
  SwerveDrive swerveDrive;
  public Drivetrain() {
  
  FieldRelative = true;
  rotationSpeed = Units.degreesToRadians(120);
  maximumSpeed = Units.feetToMeters(3);
  swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  //so basically this tells java to ignore the error so the code works!
  //dw about it

  try {
     swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }
  catch (Exception e) {
      //ignore 
  }
 
  } //end of constructor btw

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void basicDrive () {
    
    double leftX = RobotContainer.driverController.getLeftX();
    double rightX = RobotContainer.driverController.getRightX();
    double leftY = RobotContainer.driverController.getLeftY();

    double leftXV = leftX * maximumSpeed;
    double leftXY = leftY * maximumSpeed;
    Translation2d translation = new Translation2d(leftXV, leftXY);
    double rotationF = rightX * rotationSpeed;
  
    swerveDrive.drive(translation, rotationF, FieldRelative, true); 
  }

  public Command drive(){
    return Commands.runOnce(() -> {basicDrive();}, this);
  }
  
}
