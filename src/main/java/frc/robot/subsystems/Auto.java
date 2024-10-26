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

import frc.robot.Constants.DriveConstants;

public class Auto extends SubsystemBase {

  public SequentialCommandGroup autoCommandGroup;

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

  boolean justScored;
  boolean isCurrentPathValid;
  boolean hasNote;
  /* CHARS */
  public char previous = 'X';
  public char current;
  public char next;
  public char previous2 = 'X';
  public String pathName;
  public String pathName2;

  /* IS AUTO STRING VALID */
  public boolean isAutoStringValid;
  
  /* FEEDBACK */
  public String feedbackString; 
  public int feedbackCount;

  /* ARRAYS!!! */
  private char[] allNotes;
  private char[] closeNotes;
  public char[] scoringLocations;
  private char[] startingLocations;
  private String[] validPaths;
   
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
    autoString = "2BB";
    feedbackCount = 0;
    isAutoStringValid = true;

    validPaths = new String[]{

       //4   //A   //B   //C   //D   //E   //F   //G   //H
      "1-4","1-A","1-B","1-C","1-D","1-E","1-F","1-G","1-H", //1
      "2-4","2-A","2-B","2-C","2-D","2-E","2-F","2-G","2-H", //2
      "3-4","3-A","3-B","3-C","3-D","3-E","3-F","3-G","3-H", //3
            "4-A","4-B","4-C","4-D","4-E","4-F","4-G","4-H", //4
            "5-A","5-B","5-C","5-D","5-E","5-F","5-G","5-H", //5
                              "6-D","6-E",                   //6
                                                "7-G","7-H", //7
    
                                              
       //1   //2   //3   //4   //5   //6   //7   //A   //B   //C   //D   //E   //F   //G   //H
      "A-1","A-2","A-3","A-4","A-5",            "A-A","A-B","A-C","A-D","A-E",                    //A
      "B-1","B-2","B-3","B-4","B-5",            "B-A","B-B","B-C",            "B-F",              //B
      "C-1","C-2","C-3","C-4",                  "C-A","C-B","C-C",                  "C-G","C-H",  //C
      "D-1","D-2","D-3","D-4","D-5","D-6",      "D-A",                                            //D
      "E-1","E-2","E-3","E-4","E-5","E-6",      "E-A",                                            //E
      "F-1","F-2","F-3","F-4","F-5",                  "F-B",                                      //F
      "G-1","G-2","G-3","G-4","G-5",      "G-7",      "G-B","G-C",                                //G
      "H-1","H-2","H-3","H-4","H-5",      "H-7",            "H-C",                                //H


    };
    /* ALL STARTING LOCATIONS */
    startingLocations = new char[]{'1','2','3'};

    /* ALL SCORING LOCATIONS */
    scoringLocations = new char[]{'6','7','4','5','1','2','3'};

    /* ALL NOTES */
    allNotes = new char[]{'A','B','C','D','E','F','G','H'};

    /* ALL CLOSE NOTES */
    closeNotes = new char[]{'A','B','C'};

  
    feedbackString = "Errors in your input: \n";

    boolean isAutoValid = autoSequencer();
    Shuffleboard.getTab("Driver Station").add("Is auto valid:", isAutoValid).withPosition(1,2);
    Shuffleboard.getTab("Driver Station").add("Feedback:", feedbackString).withSize(5,1);


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

  /*
   * THIS METHOD CHECKS IF THE PATH IS ABLE TO BE ACCESSED
   * 
   */

   public boolean isValid(String path){
    boolean isValidPath = false;
    for(String i : validPaths){
      if(i.equals(path)) isValidPath = true;
    }
    return isValidPath;
   }

  /* SETS THE FEEDBACK */
  public void setFeedbackString(String input) {
    feedbackCount++;
    feedbackString += "\n" + ("" + feedbackCount) + ". " + input;
  }

  /*
     * 
     * 
     * ALL AUTO COMMANDS CAN BE FOUND HERE ->
     * 
     * 
     */


    /* METHOD TO ADD PATHS AS COMMANDS TO THE AUTO COMMAND GROUP */
    public void addPathToGroup (String pathString){
    RobotContainer.auto.autoCommandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathString)));
  }

  /** FOLLOWS AN AUTO PATH 
   * @param pathString this is a pathstring file name
   * i.e. (A-B)
  */
  public Command followPath(String pathString) {
      if(pathString.charAt(0) != pathString.charAt(2)) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
        return Commands.runOnce(() -> {AutoBuilder.followPath(path);}, RobotContainer.drivetrain);
      }
      else return Commands.runOnce(() -> {});
  }
  
  /* FOLLOWS PATH AND THEN INTAKES */
  public Command followPathAndIntake(String pathString){
      return Commands.parallel(
        followPath(pathString),
        RobotContainer.intakecommands.intakeAuto()
      )
      .until(RobotContainer.intake.noteInTransport());
  }

  /* FOLLOWS PATH AND SCORES RESPECTIVELY */
  /*
   * this will score amp if needed or speaker if need idk if this is going to work
   */
  public Command followPathAndShoot(String pathString, boolean isScoringAmp){
    if(!isScoringAmp){
      return Commands.sequence(
        followPath(pathString), 
        RobotContainer.drivecommands.scoringMode(false, true));
    } else {
      return Commands.sequence(
        followPath(pathString), 
        RobotContainer.shootercommands.scoreAmp());
    }
  }

  /* SEQUENTIAL COMMAND GROUP TO BE RUN BY THE ROBOT DURING AUTO */
  /*
   * 
   * WOOOOO very importante
   */
  public SequentialCommandGroup runAutoSequentialCommandGroup(){
    return RobotContainer.auto.autoCommandGroup;
  }

  /*
   * 
   * AUTOSEQUENCER
   * 
   */
  public boolean autoSequencer() {

    if(autoString.length() > 0) {
    if(isIn(autoString.charAt(0), startingLocations)) ;
    else {
      setFeedbackString("\"" + autoString.charAt(0) + "\"" + " is not a real starting location");
      isAutoStringValid = false;
    }
  }

    //isAutoStringValid = true;

    for(int i=0; i<autoString.length()-1; i++){
 
      current = autoString.charAt(i);
      //System.out.println(i + " " + current );
      if(i != 0) previous = autoString.charAt(i-1);
      else previous = 'X';

      if(i>1) previous2 = autoString.charAt(i-2);
      else previous2 = 'X';
      
      if(i < autoString.length() -1) next = autoString.charAt(i+1);
      else next = 'X';
    
      pathName = current + "-" + next;

      isCurrentPathValid = isValid(pathName);
      if(!isCurrentPathValid) {
        setFeedbackString("The path " + pathName + " is not a valid path");
        isAutoStringValid = false;
      } 


      isFromScoringLocation = isIn(current, scoringLocations);
      isFromCloseNote = isIn(current, closeNotes);
      isFromNote = isIn(current, allNotes);

      isGoingToScoringLocation = isIn(next, scoringLocations);
      isGoingToCloseNote = isIn(next, closeNotes);
      isGoingToNote = isIn(next, allNotes);

      isGoingToNoteScoringLocation = isGoingToCloseNote && isFromNote;
      isFromNoteScoringLocation = isFromCloseNote && isIn(previous, allNotes);

      shouldIntake = isGoingToNote && (isFromScoringLocation || isFromNoteScoringLocation);
      if(current == next) shouldIntake = false;
      shouldShoot = isFromNote && (isGoingToScoringLocation || isGoingToNoteScoringLocation);
      if(justScored) shouldShoot = false;


      // System.out.println("" + "\t"+ previous + "\t"+ current + "\t"+ next);
      // System.out.println("Is from scoring location? " + isFromScoringLocation);
      // System.out.println("Is from close note? "+ isFromCloseNote);
      // System.out.println("Is from note? " + isFromNote);
      // System.out.println("is the previous a note? " + isIn(previous, allNotes));
      // System.out.println("\n");
      // System.out.println("Is going to scoring location? " + isGoingToScoringLocation);
      // System.out.println("Is going to close note? "+ isGoingToCloseNote);
      // System.out.println("Is going to note? " + isGoingToNote);
      // System.out.println("\n");
      // System.out.println("Is going to note scoring location? " + isGoingToNoteScoringLocation);
      // System.out.println("Is from note scoring location? " + isFromNoteScoringLocation);
      // System.out.println("\n");
      // System.out.println("Should shoot? " + shouldShoot);
      // System.out.println("Should intake? " + shouldIntake);
      // System.out.println("just scored? " + justScored);
      //System.out.println("has a note? " + hasNote);

      //System.out.println("From scoring location: " + isFromScoringLocation);
      //System.out.println("Going to scoring location: " + isGoingToScoringLocation);

      




      
      System.out.println(pathName + "\t");

      //builds the sequential command group
      if(isAutoStringValid && isValid(pathName)){
        if(i == 0) {
          autoCommandGroup = new SequentialCommandGroup(RobotContainer.shootercommands.scoreSubwoofer());
          System.out.println("The robot will score subwoofer at the scoring location of \"" + autoString.charAt(0) + "\"\n");
          justScored = true;
        }
        if(shouldIntake){
          autoCommandGroup.addCommands(this.followPathAndIntake(pathName));
          System.out.println("The robot will follow the path \"" + pathName + "\" and intake the note at \"" + next + "\"\n");
          hasNote = true;
          justScored = false;
        } 
        else if(shouldShoot){
          if(current == next) {
            autoCommandGroup.addCommands(RobotContainer.drivecommands.scoringMode(false, true));
            if(isGoingToNoteScoringLocation) System.out.println("The robot will score using scoring mode at the note scoring location at \"" + next + "\"\n");
            else System.out.println("The robot will score using scoring mode at the scoring location \"" + next + "\"\n");
            justScored = true;
            //hasNote = false;
          }
          else if(next == '4') {
            autoCommandGroup.addCommands(this.followPathAndShoot(pathName, true));
            justScored = true;
            //hasNote = false;
          }
          else {
            autoCommandGroup.addCommands(this.followPathAndShoot(pathName, false));
            System.out.println("The robot will follow the path \"" + pathName + "\" and will shoot using scoring mode afterwards \n");
            justScored = true;
            //hasNote = false;
          }
        }

        
        //for 2BBCC
        //adds scoresubwoofer
        //adds follow path and intake
        //adds scoring mode
        //adds follow path and intake
        //adds scoring mode
        //then stops

        /*
       * 
       * validator part
       * 
       */
        

        if (isFromNote && isGoingToNote){
        if (!justScored && !isFromNoteScoringLocation) {
          isAutoStringValid = false;
          setFeedbackString("Don't try and intake the note at \"" + next + "\" when you intaked the note at \"" +
          current + "\" because you already have a note.");
        } 
        }
      
      if ((isFromNoteScoringLocation || isFromScoringLocation) && isGoingToScoringLocation && justScored &&!hasNote ){  
        isAutoStringValid = false;
        setFeedbackString("Don't try to go between the scoring location \"" + current + "\" and the scoring location \"" +
        next + "\" because you will not have another note to score.");   
      }

      if(shouldShoot) hasNote = false;
      
      
      }
    
   }

   if(isAutoStringValid) feedbackString = "Looks Good!";
  System.out.println("feedback: " + feedbackString +"\n");
  System.out.println("is the string valid:" + isAutoStringValid +"\n");

   feedbackCount = 0;
   return isAutoStringValid; 
  
  } /* END OF AUTOSEQUENCER */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
