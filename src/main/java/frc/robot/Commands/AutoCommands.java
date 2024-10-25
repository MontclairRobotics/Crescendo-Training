package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoCommands extends Command{

    /*
     * 
     * 
     * ALL AUTO COMMANDS CAN BE FOUND HERE ->
     * 
     * 
     */


    /* CREATES COMMAND GROUP FOR AUTO */
    public static SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();

    /* METHOD TO ADD PATHS AS COMMANDS TO THE AUTO COMMAND GROUP */
    public void addPathToGroup (String pathString){
    autoCommandGroup.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathString)));
  }

  /** FOLLOWS AN AUTO PATH 
   * @param pathString this is a pathstring file name
   * i.e. (A-B)
  */
  public Command followPath(String pathString) {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
      return Commands.runOnce(() -> {AutoBuilder.followPath(path);}, RobotContainer.drivetrain);
  }
  
  /* FOLLOWS PATH AND THEN INTAKES */
  public Command followPathAndIntake(String pathString){
      return Commands.sequence(
        followPath(pathString),
        RobotContainer.intakecommands.intake()
      );
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
    return AutoCommands.autoCommandGroup;
  }
}
