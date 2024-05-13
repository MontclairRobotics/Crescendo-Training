// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  /** Creates a new SwerveDrive. */

  //instance variables, which are later set to values in the constructor
  //or not if thats not applicable
  public Pigeon2 gyro;
  public boolean FieldRelative;
  public double rotationSpeed;
  public static double maximumSpeed;
  public File swerveJsonDirectory;
  public SwerveDrive swerveDrive;
  public Translation2d centerOfRotationMeters;

  //constructor
  public Drivetrain() {

  //this makes it so the robot drives with respect to the field
  FieldRelative = true;

  //how fast we rotate
  rotationSpeed = Units.degreesToRadians(120);

  //how fast we translate
  maximumSpeed = Units.feetToMeters(3);

  //accesses all of the stuff in the config files
  //these include motor ports, offsets, inverts, etc
  //the magic behind our swerve drive
  swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

  //This tells java to ignore the error so the code works!
  try {
    //it will try this code, which technically throws an error
     swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }
  catch (Exception e) {
      //but, since there isn't code to catch, it will act like
      //everything is fine and ignore the error
  }
  } //end of constructor

  //If we ever want to drive robotRelative here is the method that does so
  public void toRobotRelative() {
    FieldRelative = false;
  }
  //zeros the gyro 
  public Command zeroGyro (){
    return Commands.runOnce(() -> swerveDrive.zeroGyro(), this);
  }
  //to robotrelative but as a command
  public Command toRobotRelativeCommand() {
    return Commands.runOnce(() -> toRobotRelative(), this);
  }
  //to fieldrelative driving method
  public void toFieldRelative() {
    FieldRelative = true;
  }
  //^^ but as a command
  public Command toFieldRelativeCommand (){
    return Commands.runOnce(() -> toFieldRelative(), this);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.getPose();
  }
 
  public void drive () {
    
    //gets the input from the driverController and sets it to our xInput, yInput, and rotational input
    //they are multiplied by negative 1 because we have to invert it for some reason
    double xInput = -1*RobotContainer.driverController.getLeftX();
    double yInput = -1 *RobotContainer.driverController.getLeftY();
    double rotationInput = -1 *RobotContainer.driverController.getRightX();
    
    //Math.signum returns -1 or positive 1 based on if the value of what you input is negative or positive
    //because we are squaring the input for smoother driving, the input with read always positive
    //Math.signum fixes this issues by resetting it to negative if it is negative
    //without this, we would not be able to drive backwards or to the left or rotate one of the directions
    xInput = Math.signum(xInput) * xInput * xInput;
    yInput = Math.signum(yInput) * yInput * yInput;
    rotationInput = Math.signum(rotationInput) * rotationInput * rotationInput;

    //Simple way to create a deadband
    //if the input from the controller is so little, it will set it to zero to protect the motors
    if(Math.abs(xInput)<.04){
      xInput = 0;
    }
    if(Math.abs(rotationInput)<.04){
      rotationInput = 0;
    }
    if(Math.abs(yInput)<.04){
      yInput = 0;
    }
    //this converts our input from the controller (with all the calculations done to it)
    //into a velocity
    double xVelocity = xInput * maximumSpeed;
    double yVelocity = yInput * maximumSpeed;
    //f is for funsies!
    double rotationF = rotationInput * rotationSpeed;

    //the drive method takes in a translation2d so we use our xVelocity and yVelocity
    //to create a new Translation2d
    Translation2d translation = new Translation2d(xVelocity, yVelocity);

    //finally, we call the method that actually drives the robot using all of the code above
    swerveDrive.drive(translation, rotationF, FieldRelative, true); 
  }

  //command for the drive method
  //this is later set as the default command for the drivetrain
  //that means it will run as long as no other command is occupying the drivetrain.
  public Command driveCommand(){
    return Commands.runOnce(() -> {drive();}, this);
   
  }
 
  }
