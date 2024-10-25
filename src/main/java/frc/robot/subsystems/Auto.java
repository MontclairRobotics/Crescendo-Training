// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.AutoCommands;
import frc.robot.Constants.DriveConstants;

public class Auto extends SubsystemBase {


  /* INSTANCE VARIABLES */
  public String autoString;
  public String testAutoString;

  /* NOTE BOOLEANS */
  public boolean isFromNoteScoringLocation;
  public boolean isFromScoringLocation;
  public boolean isFromCloseNote;
  public boolean isFromNote;

  /* MORE NOTE BOOLEANS */
  public boolean isGoingToNoteScoringLocation;
  public boolean isGoingToScoringLocation;
  public boolean isGoingToCloseNote;
  public boolean isGoingToNote;
    
  /* INTAKE AND SHOOT BOOLEANS */
  public boolean shouldIntake;
  public boolean shouldShoot;

  /* CHARS */
  public char previous = 'X';
  public char current;
  public char next;
  public String pathName;

  /* IS AUTO STRING VALID */
  public boolean isAutoStringValid;
  
  /* FEEDBACK */
  public String feedback; 

  /* ARRAYS!!! */
  private char[] allNotes;
  private char[] closeNotes;
  public char[] scoringLocations;
  private char[] startingLocations;
  private char[] farNotes;
   
  /* SET TO 0,0 BECAUSE OUR ROBOT IS SQUARE */
  Translation2d centerOfRotationMeters;

  /* RETURNS POSE2D OF THE ROBOT */
  public Pose2d getPose2d(){
    return RobotContainer.drivetrain.swerveDrive.getPose();
  }

  /* RESETS POSE2D */
  public void resetPose2d (Pose2d pose){
    RobotContainer.drivetrain.swerveDrive.resetOdometry(pose);
  }

  /* DRIVES ROBOT WITH CHASSIS SPEEDS */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    RobotContainer.drivetrain.swerveDrive.drive(chassisSpeeds, false, centerOfRotationMeters);
   }

  /* RETURNS CHASSIS SPEEDS */
  public ChassisSpeeds getChassisSpeeds(){
    return RobotContainer.drivetrain.swerveDrive.getRobotVelocity();
  }

  /*
   * 
   * 
   * CONSTRUCTOR
   * 
   * 
   */
  public Auto() {

    testAutoString = "111";
    autoString = "2B";

    /* ALL STARTING LOCATIONS */
    startingLocations = new char[]{'1','2','3'};

    /* ALL SCORING LOCATIONS */
    scoringLocations = new char[]{'6','7','4','5','1','2','3'};

    /* ALL NOTES */
    allNotes = new char[]{'A','B','C','D','E','F','G','H'};

    /* ALL CLOSE NOTES */
    closeNotes = new char[]{'A','B','C'};

    /* ALL FAR NOTES */
    farNotes = new char[]{'D','F','H','I','J'};

    feedback = "Enter an autostring";

    boolean isAutoValid = autoSequencer();
    Shuffleboard.getTab("Driver Station").add("Is auto valid:", isAutoValid).withPosition(1,2);
    Shuffleboard.getTab("Driver Station").add("Feedback:", feedback).withSize(5,1);


    /* sets offset center of rotation meters to 0 */
    centerOfRotationMeters = new Translation2d(0,0);

    /* CONFIGURES AUTO SO IT ACTUALLY WORKS */
      AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(8.12, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(.01, 0.0, 0.07), // Rotation PID constants
                    RobotContainer.drivetrain.maximumSpeed, // Max module speed, in m/s
                    DriveConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );  // public Auto auto = new Auto();


  }

  /*
   * 
   * 
   * METHODS
   * 
   *
   */

  /*
   * this method takes in an array and a character you want to check,
   * and spits out a boolean if that character is in the array
   */
  public boolean isIn (char input, char[] array){

    boolean isIn = false;

    for(int i=0; i<array.length; i++){
      if(input == (array[i])){
        isIn = true;
      }
    }
    return isIn;
  }

  /* SETS THE FEEDBACK */
  public void setFeedback(String theFeedbackInput) {
    feedback = theFeedbackInput;
  }

  /*
   * 
   * AUTOSEQUENCER
   * 
   */
  public boolean autoSequencer() {
    if(autoString.length() > 0) {
    if(isIn(autoString.charAt(0), startingLocations)) AutoCommands.autoCommandGroup.addCommands(RobotContainer.shootercommands.scoreSpeaker(true));
    else {
      setFeedback("That's not a real starting location");
      isAutoStringValid = false;
      return isAutoStringValid;
    }
  }

    isAutoStringValid = true;

    for(int i=0; i<autoString.length(); i++){
 
      current = autoString.charAt(i);
      System.out.println(i + " " + current );
      if(i != 0) previous = autoString.charAt(i-1);
      else previous = 'X';
      
      if(i < autoString.length() -1) next = autoString.charAt(i+1);
      else next = 'X';
    
      pathName = current + "-" + next;

      isFromScoringLocation = isIn(current, scoringLocations);
      isFromCloseNote = isIn(current, closeNotes);
      isFromNote = isIn(current, allNotes);
      // isFromFarNote = isIn(current, farNotes);

      isGoingToScoringLocation = isIn(next, scoringLocations);
      isGoingToCloseNote = isIn(next, closeNotes);
      isGoingToNote = isIn(next, allNotes);
      // isGoingToFarNote = isIn(next, farNotes);

      isGoingToNoteScoringLocation = isGoingToCloseNote && isFromNote;
      isFromNoteScoringLocation = isFromCloseNote && isIn(previous, allNotes);

      shouldIntake = isGoingToNote && (isFromScoringLocation || isFromNoteScoringLocation);
      shouldShoot = isFromNote && (isGoingToScoringLocation || isGoingToNoteScoringLocation);

      System.out.println("From scoring location: " + isFromScoringLocation);
      System.out.println("Going to scoring location: " + isGoingToScoringLocation);
      //auto string validator part
      if (isFromNote && isGoingToNote){
        if (!isGoingToNoteScoringLocation) {
          isAutoStringValid = false;
          setFeedback("Don't go between two notes when you are not scoring at one of them.");
        } 
      }

      if (isFromScoringLocation && isGoingToScoringLocation ){
        isAutoStringValid = false;
        setFeedback("Don't go between two scoring locations");   
      }
      
      //builds the sequential command group
      if(isAutoStringValid){
        feedback = "looks good!";
        if(shouldIntake){
          AutoCommands.autoCommandGroup.addCommands(RobotContainer.autocommands.followPathAndIntake(pathName));
        } 
        if(shouldShoot){
          if(current == next) AutoCommands.autoCommandGroup.addCommands(RobotContainer.shootercommands.scoreSpeaker(true));
          else if(next == '4') AutoCommands.autoCommandGroup.addCommands(RobotContainer.autocommands.followPathAndShoot(pathName, true));
          else AutoCommands.autoCommandGroup.addCommands(RobotContainer.autocommands.followPathAndShoot(pathName, false));
        }
        // setFeedback("Looks Good!", true);
      }
    
   } 
   
  System.out.println("feedback: " + feedback);
  System.out.println("is the string valid:" + isAutoStringValid);
   return isAutoStringValid; 
  
  } /* END OF AUTOSEQUENCER */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
