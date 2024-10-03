// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  //the string that is inputed to the driverstation by the user

  public static String autoString;
  public String testAutoString;

  public boolean isFromNoteScoringLocation;
    public boolean isFromScoringLocation;
    public boolean isFromCloseNote;
    public boolean isFromNote;
    // boolean isFromFarNote;

    public boolean isGoingToNoteScoringLocation;
    public boolean isGoingToScoringLocation;
    public boolean isGoingToCloseNote;
    public boolean isGoingToNote;
    // boolean isGoingToFarNote;
    
    boolean shouldIntake;
    boolean shouldShoot;

    char previous = 'X';
    char current;
    char next;
    String pathName;


  public boolean isAutoStringValid;
  
  //the string that is sent to the driver as their feedback
  String feedback; 

  //instance variables of the arrays used in the autoSequencer
  private char[] allNotes;
  private char[] closeNotes;
  public char[] scoringLocations;
  private char[] startingLocations;
  private char[] farNotes;

  //  These are all of the methods and variables used in the configureHolonomic method,
  //  which allows us to drive autonomously by supplying the methods that the code
  //  needs to follow paths
   
  // This variable is set to (0,0) because our robot is square and so the center is not offset
  Translation2d centerOfRotationMeters;

  // This method returns the current xPosition, yPosition, and rotation2d of the robot
  public Pose2d getPose2d(){
    return RobotContainer.drivetrain.swerveDrive.getPose();
  }

  // This method takes in a pose2d which is in the form of xPosition, yPosition, and rotation2d,
  // and it resets the odometry to that pose. The odometry is where the robot THINKS it is.
  public void resetPose2d (Pose2d pose){
    RobotContainer.drivetrain.swerveDrive.resetOdometry(pose);
  }

  // This method takes in ChassisSpeeds, which are a set of xSpeeds, ySpeeds, and rotation.
  // It drives the robot with these speeds with respect to the robot (robot relative)
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    RobotContainer.drivetrain.swerveDrive.drive(chassisSpeeds, false, centerOfRotationMeters);
   }

   //This method returns the current ChassisSpeeds of the robot.
  public ChassisSpeeds getChassisSpeeds(){
    return RobotContainer.drivetrain.swerveDrive.getRobotVelocity();
  }

  /** Creates a new Auto. */
  public Auto() {

     testAutoString = "111";
     autoString = "2BBAAC5";

    //creates the array of all STARTING locations
    startingLocations = new char[]{'1','2','3'};

    //creates the array of all SCORING locations
    scoringLocations = new char[]{'4','5','1','2','3'};

    //creates the array of all the notes
    allNotes = new char[]{'A','B','C','D','E','F','G','H'};

    //creates the array of all close notes
    closeNotes = new char[]{'A','B','C'};

    //creates the array of all far notes
    farNotes = new char[]{'D','F','H','I','J'};

    //creates the input for the autoString; i already tried but i gave up
    //TOdO: figure out how to recieve a text entry

    feedback = "f";

    boolean isAutoValid = autoSequencer();
    Shuffleboard.getTab("Driver Station").add("Is auto valid:", isAutoValid).withPosition(1,2);
    Shuffleboard.getTab("Driver Station").add("Feedback:", feedback).withSize(5,1);


    //sets the value of the offset (again, since our drivetrain is square, this is zero)
    centerOfRotationMeters = new Translation2d(0,0);
      AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetPose2d, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(8.12, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(.01, 0.0, 0.07), // Rotation PID constants
                    Drivetrain.maximumSpeed, // Max module speed, in m/s
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


  }//end of constructor


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

  //creates the command group for the auto
  public static SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();

  //method to easily add paths to a sequential command group
  public void addPathToGroup (String pathString){
    autoCommandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathString)));
  }
  public void followPathAndIntakeMethod (String pathString) {
    AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathString));
    RobotContainer.intake.inhaleCommand();
  }
  public Command followPathCommand(String pathString) {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
      return Commands.runOnce(() -> {AutoBuilder.followPath(path);}, this, RobotContainer.drivetrain);
  }
  public Command followPathAndIntakeCommand(String pathString){
      return Commands.runOnce(() -> followPathAndIntakeMethod(pathString), this, RobotContainer.intake, RobotContainer.drivetrain);
  }
  public Command followPathAndShootCommand(String pathString, boolean isScoringAmp){
    if(!isScoringAmp){
      return Commands.runOnce(() -> followPathCommand(pathString), this, RobotContainer.drivetrain).andThen(() -> RobotContainer.shooter.shootSpeakerCommand(), this, RobotContainer.shooter);
    } else {
      return Commands.runOnce(() -> followPathCommand(pathString), this, RobotContainer.drivetrain).andThen(() -> RobotContainer.shooter.shootAmpCommand(), this, RobotContainer.shooter);
    }
  }
  //TODO: load path file when building sequence 
  //add .follow path to sequence instread of .followpath command
  //add try catch bllock.

  public void setFeedback(String theFeedbackInput) {
    feedback = theFeedbackInput;
  }

  //AUTOSEQUENCER
  public boolean autoSequencer() {
    
    //TODO: figure out why this isnt working
    if(isIn(autoString.charAt(0), startingLocations)) autoCommandGroup.addCommands(RobotContainer.shooter.shootSpeakerCommand());
    else {
      setFeedback("That's not a real starting location");
      isAutoStringValid = false;
    }

    isAutoStringValid = true;

    for(int i=0; i<autoString.length(); i++){
 
      current = autoString.charAt(i);
      System.out.println(i + " " + current );
      if(i != 0) previous = autoString.charAt(i-1);
      
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

      System.out.println("From: " + isFromScoringLocation);
      System.out.println("Going to: " + isGoingToScoringLocation);
      //auto string validator part
      if(isFromNote && isGoingToNote){
        if(!isGoingToNoteScoringLocation) {
          isAutoStringValid = false;
          setFeedback("Don't go between two notes when you are not scoring at one of them.");
      } 
      }

      if(isFromScoringLocation   &&  isGoingToScoringLocation ){
        isAutoStringValid = false;
        setFeedback("Don't go between two scoring locations");      
      }
      
      //builds the sequential command group
      if(isAutoStringValid){
        feedback = "looks good";
        if(shouldIntake){
          autoCommandGroup.addCommands(followPathAndIntakeCommand(pathName));
        } 
        if(shouldShoot){
          if(current == next) autoCommandGroup.addCommands(RobotContainer.shooter.shootSpeakerCommand());
          else if(next == '4') autoCommandGroup.addCommands(followPathAndShootCommand(pathName, true));
          else autoCommandGroup.addCommands(followPathAndShootCommand(pathName, false));
        }
        // setFeedback("Looks Good!", true);
      }
    

    // System.out.println("the auto sequencer finished");
    //return statement
   } 
   
  System.out.println("feedback: "+ feedback);
  System.out.println("is the string valid:" + isAutoStringValid);
   return isAutoStringValid; 
  
  } //end of is AutoStringValid
  

  public SequentialCommandGroup runAutoSequentialCommandGroup(){
    return autoCommandGroup;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("hi")
  }

}
