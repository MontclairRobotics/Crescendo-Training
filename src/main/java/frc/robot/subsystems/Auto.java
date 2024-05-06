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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class Auto extends SubsystemBase {

  //the string that is inputed to the driverstation by the user
  String autoString;

  //the string that is sent to the driver as their feedback
  String feedback; 

  //instance variables of the arrays used in the autoSequencer
  private char[] allNotes;
  private char[] closeNotes;
  private char[] scoringLocations;
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

    //creates the array of all STARTING locations
    startingLocations = new char[]{'1','2','3'};

    //creates the array of all SCORING locations
    scoringLocations = new char[]{'A','B','C','4','5','1','2','3'};

    //creates the array of all the notes
    allNotes = new char[]{'A','B','C','D','E','F','G','H'};

    //creates the array of all close notes
    closeNotes = new char[]{'A','B','C'};

    //creates the array of all far notes
    farNotes = new char[]{'D','F','H','I','J'};

    //creates the input for the autoString; i already tried but i gave up
    //TOdO: figure out how to recieve a text entry
    autoString = "";

    feedback = "Looks good!";

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
    );

  }//end of constructor

  public Auto auto = new Auto();

  /*
   * this method takes in an array and a character you want to check,
   * and spits out a boolean if that character is in the array
   */
  public boolean isIn (char[] array, char input){

    boolean isIn = false;

    for(int i=0; i<array.length; i++){
      if(input == (array[i])){
        isIn = true;
      }
    }
    return isIn;
  }

  //sets the feedback for the driver which will be 
  public void setFeedback(String theFeedbackInput) {
      feedback = theFeedbackInput;
  }

  //creates the command group for the auto
  public static SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup(null);

  //method to easily add paths to a sequential command group
  public void addPathToGroup (String pathString){
    autoCommandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathString)));
  }


  //AUTOSEQUENCER
  public boolean autoSequencer() {

    //some variables that will be used later
    boolean isAutoStringValid = true;
    char current;
    char next;

    //checks if the first character of autoString is a valid starting spot
    if(isIn(startingLocations, autoString.charAt(0))){
      //nothing here because we want the first character of autoString to be a startinglocation
    } else {
      isAutoStringValid = false;
      feedback = "That's not a real starting location";
    }

    
    // a big for loop that checks the rest of the autoString for errors
    for(int i=0; i<autoString.length(); i++){

      //sets certain variables to the current character and next character in the autoString
      //its for useful purposes
      current = autoString.charAt(i);
      next = autoString.charAt(i+1);

    
      
      
    }

    //return statement
    return isAutoStringValid;
  } //end of is AutoStringValid

  public void runAutoSequentialCommandGroup(){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
