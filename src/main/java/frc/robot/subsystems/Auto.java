// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
// import java.util.EnumSet;
// import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEvent.Kind;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
// import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Auto extends SubsystemBase {

  public String[] pathsToLoad;
  /* list to hold all of the preloaded paths needed to be loaded before running autonomous */
  private List<PathPlannerPath> preloadedPaths;

  /* command group to be run for the auto */
  public SequentialCommandGroup autoCommandGroup;

  /* this recieves the value for the autostring of type generic entry */
  public GenericEntry autoStringInput;

  /* declares variables to hold the current and previous autoString */
  public String autoString;
  public String previousAutoString;

  /* booleans for the autosequencer */
  public boolean isFromNoteScoringLocation; //checks if it just scored at a note scoring location
  public boolean isFromScoringLocation; //checks if it is from a regular scoring location or note scoring location
  public boolean isFromCloseNote; //checks if it is from a close note
  public boolean isFromNote; //checks if it is from any note
  public boolean isGoingToNoteScoringLocation; //checks if it is about to score at note scoring location
  public boolean isGoingToScoringLocation; //checks if it is about to score at any scoring location
  public boolean isGoingToCloseNote; //checks if it is going to a close note
  public boolean isGoingToNote; //checks if it is going to any note
    
  /* these booleans hold values to see if the robot should shoot or intake
   * based on the booleans declared above */
  public boolean shouldIntake;
  public boolean shouldShoot;
  
  /* booleans to see whether the robot just scored or if it has a note
   * based on which commands have been added to the autoCommandGroup */
  public boolean justScored;
  public boolean hasNote;

  /* boolean to store whether or not the current path is valid */
  public boolean isCurrentPathValid;

  /* chars to store relative parts of the autoString */
  public char previous2 = ' '; //protecting for domain errors
  public char previous = ' '; //protecting for domain errors
  public char current;
  public char next;

  /* this is a String to hold the current pathName to check if it is valid
   * also used for loading and following paths from their given pathString */
  public String pathName;

  /* this is a boolean to see if the autoString has changed previously
   * if this is the case, then autoSequencer will run */
  public boolean hasAutoStringChanged;

  /* boolean to store whether or not the autoString is Valid */
  public boolean isAutoStringValid;
  
  /* feedback strings to be send to shuffleboard */
  public String feedbackString; 
  public String feedbackString2;
  
  /* strings to be sent to shuffleboard that roughly describes
   * the sequence of commands being run in autonomous */
  public String commandString;
  public String commandString2;

  /* declaration of all the arrays*/
  private char[] allNotes; //all notes
  private char[] closeNotes; //all close notes
  private char[] scoringLocations; //all scoring locations **not including note scoring locations
  private char[] startingLocations; //all valid starting locations
  private String[] validPaths; //all valid paths
  private char[] intakedNotes; //all the notes we've intaked at
  private char[] notesScoredAt; //all the notes we've scored at
   
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
    RobotContainer.drivetrain.swerveDrive.drive(chassisSpeeds, true, centerOfRotationMeters);
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

    /* sets offset center of rotation meters to 0 */
    centerOfRotationMeters = new Translation2d(0,0);

    //sets the values of the feedback and command strings as to not throw NullExceptionPoint errors
    commandString = "";
    commandString2 = "";
    feedbackString = "Looks Good!";
    feedbackString2 = "";

    //sets the previousAutoString and autoStringInput
    //(autoString input is converted into a String in the periodic method; right now it is a generic entry)
    previousAutoString = "";
    autoStringInput = Shuffleboard.getTab("Driver Station").add("Auto String Entry", "").withPosition(0,0).getEntry();
    
    /* autoString is valid by default */
    isAutoStringValid = true;

    /* ARRAY OF VALID PATHS */
    validPaths = new String[]{

              //4   //A   //B   //C   //D   //E   //F   //G   //H
        /*1*/ "1-4","1-A","1-B","1-C","1-D","1-E","1-F","1-G","1-H", 
        /*2*/ "2-4","2-A","2-B","2-C","2-D","2-E","2-F","2-G","2-H", 
        /*3*/ "3-4","3-A","3-B","3-C","3-D","3-E","3-F","3-G","3-H", 
        /*4*/       "4-A","4-B","4-C","4-D","4-E","4-F","4-G","4-H", 
        /*5*/       "5-A","5-B","5-C","5-D","5-E","5-F","5-G","5-H", 
        /*6*/                         "6-D","6-E",                   
        /*7*/                                           "7-G","7-H", 
    
                                              
              //1   //2   //3   //4   //5   //6   //7   //A   //B   //C   //D   //E   //F   //G   //H
        /*A*/ "A-1","A-2","A-3","A-4","A-5",            "A-A","A-B","A-C","A-D","A-E",                    
        /*B*/ "B-1","B-2","B-3","B-4","B-5",            "B-A","B-B","B-C",            "B-F",              
        /*C*/ "C-1","C-2","C-3","C-4",                  "C-A","C-B","C-C",                  "C-G","C-H",  
        /*D*/ "D-1","D-2","D-3","D-4","D-5","D-6",      "D-A",                                            
        /*E*/ "E-1","E-2","E-3","E-4","E-5","E-6",      "E-A",                                            
        /*F*/ "F-1","F-2","F-3","F-4","F-5",                  "F-B",                                      
        /*G*/ "G-1","G-2","G-3","G-4","G-5",      "G-7",      "G-B","G-C",                                
        /*H*/ "H-1","H-2","H-3","H-4","H-5",      "H-7",            "H-C",                                


    };

    /* ARRAY OF ALL STARTING LOCATIONS */
    startingLocations = new char[]{'1','2','3'};

    /* ARRAY OF ALL SCORING LOCATIONS */
    scoringLocations = new char[]{'6','7','4','5','1','2','3'};

    /* ARRAY OF ALL NOTES */
    allNotes = new char[]{'A','B','C','D','E','F','G','H'};

    /* ARRAY OF ALL CLOSE NOTES */
    closeNotes = new char[]{'A','B','C'};

    /* LOGGING PURPOSES */
    Shuffleboard.getTab("Driver Station").addBoolean("Is auto valid:", isAutoStringValidSupplier()).withPosition(1, 0);
    Shuffleboard.getTab("Driver Station").addString("Feedback:", feedbackStringSupplier()).withSize(7,  1).withPosition(2, 0);
    Shuffleboard.getTab("Driver Station").addString("Feedback (cont.)", feedbackString2Supplier()).withSize(9,  1).withPosition(0,1);
    Shuffleboard.getTab("Driver Station").addString("Commands", commandStringSupplier()).withSize(9,  1).withPosition(0, 2);
    Shuffleboard.getTab("Driver Station").addString("Commands (cont.)", commandString2Supplier()).withSize(9,  1).withPosition(0, 3);

    /* CONFIGURES AUTO SO IT ACTUALLY WORKS */
      AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetPose2d,
            this::getChassisSpeeds, // Method to reset odometry (will be called if your auto has a starting pose)
            (ChassisSpeeds x) -> {
              RobotContainer.drivetrain.getSwerveDrive().drive(new ChassisSpeeds(x.vxMetersPerSecond, x.vyMetersPerSecond, x.omegaRadiansPerSecond), false, centerOfRotationMeters);
            }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            //this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.PATH_FOLLOWER_CONFIG,
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
            RobotContainer.drivetrain
            // Rference to this subsystem to set requirements
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
    * method that checks whether the autoString changed
    * determines if the autoSequencer runs or not
    */
  public Boolean hasAutoStringChanged(){
      hasAutoStringChanged = !autoString.equals(previousAutoString);
      previousAutoString = autoString;
      return hasAutoStringChanged;
  }

  //this method gets the autoString input (type GenericEntry) as a String
  public String getAutoString(){
      return autoStringInput.getString("");
  }

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
    String s = feedbackString + "\t" + input + ".";
    if(s.length()<140 && feedbackString2.length() < 1) feedbackString += "\t" + input + ".";
    else feedbackString2 += "\t" + input + ".";
  }

  /* SETS THE COMMAND STRING */
  public void setCommandString(String input){
    
    String s = commandString + input;
    if(s.length()<180 && commandString2.length() < 1) commandString += input;
    else commandString2 += input;
  }

  //adds all of the paths to an array of path planner paths
  public void preloadPaths(String[] pathNames) {
    for (String pathName : pathNames) {
        if(isValid(pathName)){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        if (path != null) {
            preloadedPaths.add(path);
            System.out.println("Loaded path: " + pathName);
        } else {
            System.out.println("Failed to load path: " + pathName);
        }
      }
    }
  }

  /*
   * 
   * Various suppliers for logging purposes
   * 
   */
  public Supplier<String> feedbackStringSupplier(){
    return () -> feedbackString;
  }

  public Supplier<String> feedbackString2Supplier(){
    return () -> feedbackString2;
  }

  public Supplier<String> autoStringSupplier(){
    return () -> autoString;
  }

  public BooleanSupplier isAutoStringValidSupplier(){
    return () -> isAutoStringValid;
  }

  public Supplier<String> commandStringSupplier(){
    return () -> commandString;
  }

  public Supplier<String> commandString2Supplier(){
    return () -> commandString2;
  }


  /*
     * 
     * 
     * ALL AUTO COMMANDS CAN BE FOUND HERE ->
     * 
     * 
     */


  /** FOLLOWS AN AUTO PATH 
   * @param pathString this is a pathstring file name
   * i.e. (A-B)
  */
  public Command followPath(String pathString) {
    //if the path is in the preloadedPaths array, which it should be, it will find the path that equals
    //the path from the pathString and returns the command that follows that path
    for (PathPlannerPath path : preloadedPaths) {
        if (path.equals(PathPlannerPath.fromPathFile(pathString))) {
            System.out.println("\nFollowing path " + pathString + " with the preloaded path\n");
            return AutoBuilder.followPath(path);
        } 
    }
    //otherwise, it will load the path here 
    System.out.println("\nFollowing path "+ pathString + " with NOT preloaded path\n");
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
    return AutoBuilder.followPath(path);
}

  public Command oldFollowPath(String pathString) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
    return AutoBuilder.followPath(path);
  }

  /* FOLLOWS PATH AND THEN INTAKES */
  public Command followPathAndIntake(String pathString){
      return Commands.parallel(
        RobotContainer.intakecommands.intakeAuto(),
        followPath(pathString)
      )
      .withTimeout(2);
      //.andThen(RobotContainer.intakecommands.intakeAuto())
      //.onlyWhile(RobotContainer.intake.noteOutOfTransport())
     // .withTimeout(0.7)
      //.andThen(Commands.runOnce(() -> System.out.println(" \n \n\n\n FOLLOWING A PATH AND INTAKE FOR REAL\n\n\n\n")));
      
  }

  /* FOLLOWS PATH AND SCORES RESPECTIVELY */
  /*
   * this will score amp if needed or speaker if needed
   */
  public Command followPathAndShoot(String pathString, boolean isScoringAmp){
    System.out.println("\n\n\n the follow path and shoot command is being called \n\n\n");
    if(!isScoringAmp){
      return Commands.sequence(
        Commands.runOnce(() -> System.out.println("\n\n\nSCORING MODE AFTER FOLLOWING A PATH\n\n\n")),
        followPath(pathString), 
        RobotContainer.drivecommands.scoringModeAuto(false));
    } else {
      return Commands.sequence(
        followPath(pathString), 
        RobotContainer.shootercommands.scoreAmp())
        .andThen(Commands.runOnce(() -> System.out.println("\n\n\nScoring amp after following a path\n\n\n")));
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
   * 
   * 
   * 
   * 
   * 
   * 
   * AUTOSEQUENCER
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   */

  public boolean autoSequencer() {

    /* instantiates some variables that need to be reset every time autoSequencer is run */

    pathsToLoad = new String[50]; //creates the array of preload PATH STRING NAMES
    preloadedPaths = new ArrayList<>(); //creates the array of preloaded paths
    notesScoredAt = new char[50]; //we don't want notes we scored at in previous autos to carry over to a new auto
    intakedNotes = new char[50]; //same logic 
    commandString = ""; //resets the String to hold commands every time
    commandString2 = ""; //^^
    autoString = getAutoString(); //gets the value of the autoString
    autoCommandGroup = new SequentialCommandGroup(); //creates a new commandGroup each time
    feedbackString = "\n"; //resets feedback
    feedbackString2 = ""; //^^
    isAutoStringValid = true;

    //CHECKS IF STARTING LOCATION IS VALID
    if(autoString.length() > 0) {
    if(isIn(autoString.charAt(0), startingLocations)) ;
    else {
      setFeedbackString(" \"" + autoString.charAt(0) + "\"" + " is not a real starting location");
      isAutoStringValid = false;
    }
  }

  /**
   * 
   * GIANT FOR LOOP TO VALIDATE AUTOSTRING AND CREATE AUTOCOMMANDGROUP
   * 
   */

    for(int i=0; i<autoString.length()-1; i++){
 
      /* sets the values for the different relevant characters of the autoString using safeties to avoid domain errors */
      current = autoString.charAt(i);
      previous = i != 0 ? autoString.charAt(i-1) : ' ';
      previous2 = i > 1 ? autoString.charAt(i-2) : ' ';
      next = i < autoString.length()-1 ? autoString.charAt(i+1) : ' ';

      /* sets the path name */
      pathName = current + "-" + next;

      /* detects whether current path is valid or not, then sets isAutoStringValid to false */
      if(!isValid(pathName)) {
        setFeedbackString(" Invalid path (" + pathName +")");
        isAutoStringValid = false;
      } 
      
      /* instantiates all of the booleans for the autoSequencer */
      isFromScoringLocation = isIn(current, scoringLocations);
      isFromCloseNote = isIn(current, closeNotes);
      isFromNote = isIn(current, allNotes);

      isGoingToScoringLocation = isIn(next, scoringLocations);
      isGoingToCloseNote = isIn(next, closeNotes);
      isGoingToNote = isIn(next, allNotes);

      isGoingToNoteScoringLocation = isGoingToCloseNote && isFromNote;
      isFromNoteScoringLocation = isFromCloseNote && isIn(previous, allNotes);
      if(!justScored) isFromNoteScoringLocation = false;

      shouldIntake = isGoingToNote && (isFromScoringLocation || isFromNoteScoringLocation);
      if(current == next) shouldIntake = false;
      shouldShoot = isFromNote && (isGoingToScoringLocation || isGoingToNoteScoringLocation);
      if(justScored) shouldShoot = false;

      /*
       * 
       * BUILDS THE SEQUENTIAL COMMAND GROUP
       * 
       */

      if(isValid(pathName)){ //checks if path is valid
        
        if(i == 0) {
          autoCommandGroup.addCommands(RobotContainer.shootercommands.scoreSubwoofer());
          setCommandString("Scoring subwoofer. ");
          justScored = true;
          hasNote = false;
        }
        /*
         * 
         * if intaking ->
         * 
         * 
         */
        if(shouldIntake){
          //adds the intake command
          autoCommandGroup.addCommands(this.followPathAndIntake(pathName));
          setCommandString("Following " + pathName + " and intaking note " + next + ". ");
          hasNote = true;
          justScored = false;

          /* adds paths to the ones we need to load */
          pathsToLoad[i] = pathName;

          //checks if we already intake that not
          if(isIn(next, intakedNotes)){
          isAutoStringValid = false;
          setFeedbackString(" You have already intaked note " + next); 
          }

          //sets the note we just intaked into the intakedNotes array
          intakedNotes[i] = next;

          //checks if we scored at a note we tried to intake, then throws an error because that note will be bumped out of the way
          if(isIn(next, notesScoredAt)){
            isAutoStringValid = false;
            setFeedbackString(" You have previously scored at " + next + " before intaking that note and now it is bumped out of the way, so you won't be able to intake it");
          }
        } 
          /*
           * 
           * if shooting ->
           * 
           * 
           */
        else if(shouldShoot){
          //scoring mode without moving (note scoring location)
          if(current == next) { 
            autoCommandGroup.addCommands(RobotContainer.drivecommands.scoringModeAuto(false));
            
              setCommandString("Scoring mode at " + next + ". ");
              justScored = true;
              hasNote = false;
              notesScoredAt[i] = next;
    
          }
          //scoring at amp
          else if(next == '4') {
            autoCommandGroup.addCommands(this.followPathAndShoot(pathName, true));
            justScored = true;
            hasNote = false;
            setCommandString("Following " + pathName + " and scoring amp. ");
            pathsToLoad[i] = pathName;
          }
          //scoring at any other scoring location
          else {
            autoCommandGroup.addCommands(this.followPathAndShoot(pathName, false));
            justScored = true;
            hasNote = false;
            setCommandString("Following " + pathName + " and then shoot using scoring mode. ");
            notesScoredAt[i] = next;
            pathsToLoad[i] = pathName;
          }
        }

        /*
       * 
       * validator part
       * 
       */
    
      //checks if its going between two notes
        if (isFromNote && isGoingToNote){
        if (!isFromNoteScoringLocation && hasNote) {
          isAutoStringValid = false;
          setFeedbackString(" Don't try and intake the note at \"" + next + "\" when you already intaked the note at \"" +
          current + "\"");
        } 
        }
      //checks if you are going between two scoring locations
      if ((isFromNoteScoringLocation || isFromScoringLocation) && isGoingToScoringLocation && justScored &&!hasNote ){  
        isAutoStringValid = false;
        setFeedbackString(" Don't try to go between the scoring location \"" + current + "\" and the scoring location \"" +
        next + "\" because you will not have another note to score with");   
      }

      if(shouldShoot) hasNote = false;
      
      }// end of is path valid
    
   } //end of for loop

   if(isAutoStringValid) feedbackString = "Looks Good!";
   return isAutoStringValid; 
  
  } /* END OF AUTOSEQUENCER */
  

  /* 
   * PERIODIC METHOD
   * WILL BE CALLED ONCE EVERY SCHEDULER RUN
   */
  @Override
  public void periodic() {
    autoString = getAutoString();
    if(hasAutoStringChanged()) autoSequencer();
  }

}
