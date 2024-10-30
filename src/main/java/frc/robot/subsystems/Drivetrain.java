// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.vision.Limelight;

public class Drivetrain extends SubsystemBase {

  /** Creates a new SwerveDrive. */

  /* INSTANCE VARIABLES */
  public Pigeon2 gyro;
  public boolean FieldRelative;
  public double rotationSpeed;
  public double maximumSpeed;
  public File swerveJsonDirectory;
  public SwerveDrive swerveDrive;
  public Translation2d centerOfRotationMeters;
  public boolean testing;
  
  /* INSTANCE VARIABLES FOR TURNING ROBOT RELATIVE */
  public double setPoint;
  public double wrappedSetPoint;
  public double odometryHeading;
  public PIDController thetaController;
  public double response;
  public boolean isRobotAtAngleSetPoint;
  public boolean shouldStop;

  /*
   * 
   * 
   * CONSTRUCTOR
   * 
   * 
   */
  public Drivetrain() {


  FieldRelative = true;

  rotationSpeed = Units.degreesToRadians(120);

  maximumSpeed = Units.feetToMeters(3);

     /* ACCESSES ALL THE IMPORTANT INFORMATION NEEDED FOR THE SWERVE DRIVE */
  swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
     try {
    //it will try this code, which technically throws an error
     swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }
  catch (Exception e) {
      /* but, since there isn't code to catch, it will act like
      * everything is fine and ignore the error
      */
  }
  
 //to convert back, comment this out until
    // double kp = swerveDrive.getSwerveController().thetaController.getP();
    // double ki = swerveDrive.getSwerveController().thetaController.getI();
    // double kd = swerveDrive.getSwerveController().thetaController.getD();
    // thetaController = new PIDController(kp, ki, kd);

    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // thetaController.setTolerance(DriveConstants.ANGLE_DEADBAND * ((Math.PI ) / 180 ), DriveConstants.VELOCITY_DEADBAND * ((Math.PI ) / 180 ));
    // //end comment here
  /* FOR AUTO ALIGN TO ANGLE */
  //comment this in
  thetaController = new PIDController(.87, .4, 0.05);
  thetaController.setTolerance(1);
  thetaController.enableContinuousInput(-180, 180);

  }

  /*
   * 
   * 
   * METHODS
   * 
   * 
   */

  /* CONVERTS DRIVING TO ROBOT RELATIVE */
  public void toRobotRelative() {
    FieldRelative = false;
  }

  /* CONVERTS DRIVING TO FIELD RELATIVE */
  public void toFieldRelative() {
    FieldRelative = true;
  }

  /* DEFAULT DRIVE METHOD */
  public void drive () {
    
    /* RECIEVES INPUTS FROM CONTROLLER */
    double xInput = -1*RobotContainer.driverController.getLeftY();
    double yInput = -1*RobotContainer.driverController.getLeftX();
    double rotationInput = -1 *RobotContainer.driverController.getRightX();
    
    /* CUBES THE INPUT AND MAKES SURE THEY ARE THE RIGHT SIGN */
    xInput = Math.signum(xInput) * xInput * xInput;
    yInput = Math.signum(yInput) * yInput * yInput;
    rotationInput = Math.signum(rotationInput) * rotationInput * rotationInput;

    /* DEADBAND TO PROTECT MOTORS */
    if(Math.abs(xInput)<.04){
      xInput = 0;
    }
    if(Math.abs(rotationInput)<.04){
      rotationInput = 0;
    }
    if(Math.abs(yInput)<.04){
      yInput = 0;
    }
    
    /* CONVERTS THE INPUTS INTO A VELOCITY */
    double xVelocity = xInput * maximumSpeed;
    double yVelocity = yInput * maximumSpeed;
    double rotationF = rotationInput * rotationSpeed;

    /* CREATES NEW TRANSLATION BASED ON THESE VALUES */
    Translation2d translation = new Translation2d(xVelocity, yVelocity);

    /* CALL THE SWERVE DRIVE METHOD */
    swerveDrive.drive(translation, rotationF, FieldRelative, true); 

  }

  /* ROBOT RELATIVE SETPOINT METHOD */
  public void setSetpoint(double angleRadians){
    //comment the math.toRadians part out
    wrappedSetPoint = 
      wrapAngle(odometryHeading + angleRadians)
      
      ;
      setPIDRelativeTurning();
    thetaController.setSetpoint(wrappedSetPoint);
  }

  /* ALIGNS TO THE ANGLE DURING SCORING MODE */
  public void alignScoringMode(boolean lockDrive){
    if(RobotContainer.shooter.scoringMode){

    Translation2d translation = new Translation2d(0,0);

    setSetpoint(Limelight.getTX());
      setPIDScoringMode();
    response = -thetaController.calculate(odometryHeading) 
    //comment this in
    *Math.PI /180
    ;

    if(lockDrive) swerveDrive.drive(translation, response, false, true);
    else swerveDrive.drive(returnDefaultDriveTranslation(), response, true, true);
    } else stopScoringMode();
  }

  /* STOPS SCORING MODE */
  public void stopScoringMode(){
    RobotContainer.shooter.stop();
    RobotContainer.sprocket.stop();
  }

  public boolean isAlignedTX(){
    if(Limelight.getTX() < .5) return true;
    else return false;
  }
 
  /* ALIGN TO ANGLE ROBOT RELATIVE */
  public void alignToAngleRobotRelative(boolean lockDrive) {
    //creates a blank translation to pass in to the drive function so the robot doesn't move
    Translation2d translation = new Translation2d(0, 0);
    //calculates the input for the drive function (NO IDEA IF I SHOULD MULTIPLY THIS BY SOMETHING)
    //inputs field relative angle (set point is also converted to field relative)
    setPIDRelativeTurning();
    response = thetaController.calculate(odometryHeading) 
    * Math.PI / 180
    //comment this back in
    ;
    //calls the drive function with no translation but with turning
    //should work maybe idk
    if(lockDrive) swerveDrive.drive(translation, response, false, true);
    else swerveDrive.drive(returnDefaultDriveTranslation(), response, false, true);
    }

  /*
  * 
  * VARIOUS LOGGING TOOLS 
  *
  */
  public void setPIDScoringMode(){
    thetaController.setP(DriveConstants.P2);
    thetaController.setI(DriveConstants.I2);
    thetaController.setD(DriveConstants.D2);
  }

  public void setPIDRelativeTurning(){
    thetaController.setP(DriveConstants.P1);
    thetaController.setI(DriveConstants.I1);
    thetaController.setD(DriveConstants.D1);
  }

  public BooleanSupplier isRobotNOTAtAngleSetPoint(){
    return () -> !isRobotAtAngleSetPoint;
  }
  public DoubleSupplier responseSupplier(){
    return () -> response;
  }
  public DoubleSupplier odometryHeadingDoubleSupplier(){
    return () -> odometryHeading;
  }
  public BooleanSupplier isRobotAtAngleSetPoint(){
    return () -> isRobotAtAngleSetPoint;
  }
  public DoubleSupplier setPointSupplier(){
    return () -> setPoint;
  }
  public DoubleSupplier wrappedSetPointSupplier(){
    return () -> wrappedSetPoint;
  }

  /* SETS THE FIELD RELATIVE SET POINT ANGLE */
  public void setFieldRelativeAngle(double angle){
    //comment out math.toRadians
    double wrappedAngle = wrapAngle(angle);
    setPIDRelativeTurning();
    thetaController.setSetpoint(wrappedAngle);
  }

  /* ALIGNS TO ANGLE FIELD RELATIVE */
  public void alignToAngleFieldRelative(boolean lockDrive){
    Translation2d translation = new Translation2d(0,0);
    setPIDRelativeTurning();
    double response = thetaController.calculate(odometryHeading) 
    *Math.PI/180
    //comment this back in
    ;
    if(lockDrive) swerveDrive.drive(translation, response, true, true);
    else swerveDrive.drive(returnDefaultDriveTranslation(), response, true, true);
  }

  /* RETURNS THE TRANSLATION2D MADE BY JOYSTICK INPUTS */
  public Translation2d returnDefaultDriveTranslation(){
    double xInput;
    double yInput;
    double rotationInput;
    xInput = -1*RobotContainer.driverController.getLeftY();
    yInput = -1*RobotContainer.driverController.getLeftX();
    rotationInput = -1 *RobotContainer.driverController.getRightX();
    xInput = Math.signum(xInput) * xInput * xInput;
    yInput = Math.signum(yInput) * yInput * yInput;
    rotationInput = Math.signum(rotationInput) * rotationInput * rotationInput;
    if(Math.abs(xInput)<.04) xInput = 0;
    if(Math.abs(yInput)<.04) yInput = 0;
    double xVelocity = xInput * maximumSpeed;
    double yVelocity = yInput * maximumSpeed;
    Translation2d translation2d = new Translation2d(xVelocity, yVelocity);
    return translation2d;
  }

  /* WRAPS THE ANGLE BETWEEN -180 AND 180 */
  public static double wrapAngle(double angle) {
    angle = (angle + 180) % 360; // Step 1 and 2
    if (angle < 0) {
        angle += 360; // Make sure it's positive
    }
    return angle - 180; // Step 3
  }

  /* BOOLEAN SUPPLIER TO SAY THAT THERE IS INPUT */
  public BooleanSupplier inputNotIgnorable(){
    return () -> !shouldStop;
  }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    shouldStop = !isRobotAtAngleSetPoint && Math.abs(response) < 0.05;
    swerveDrive.getPose();
    //comment this get radians to getdegrees
    odometryHeading = swerveDrive.getPose().getRotation().getDegrees();
    isRobotAtAngleSetPoint = thetaController.atSetpoint();
  }
  
 
  }
