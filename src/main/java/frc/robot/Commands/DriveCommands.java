package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class DriveCommands extends Command {

    /*
     * 
     * 
     * ALL DRIVETRAIN COMMANDS CAN BE FOUND HERE ->
     * 
     * 
     */


     /* zeros the gyro */
    public Command zeroGyro() {
        return Commands.runOnce(() -> RobotContainer.drivetrain.swerveDrive.zeroGyro(), RobotContainer.drivetrain);
    } 
    
    /* turns boolean to robot relative */
    public Command toRobotRelativeCommand() {
        return Commands.runOnce(() -> RobotContainer.drivetrain.toRobotRelative(), RobotContainer.drivetrain);
      }

    /* turns boolean to field relative */
    public Command toFieldRelativeCommand (){
        return Commands.runOnce(() -> RobotContainer.drivetrain.toFieldRelative(), RobotContainer.drivetrain);
    }

    /* default command for the drivetrain */
    public Command driveCommand(){
        return Commands.run(() -> RobotContainer.drivetrain.drive(), RobotContainer.drivetrain);
      }

    /* stops scoring mode */
    public Command stopScoringModeCommand(){
        return Commands.runOnce(() -> RobotContainer.drivetrain.stopScoringMode());
    }

    /* aligns to angle robot relative */
    public Command alignToAngleRobotRelative(double angle, boolean lockDrive) {
        return Commands.sequence(
            Commands.runOnce(() -> RobotContainer.drivetrain.setSetpoint(angle), RobotContainer.drivetrain),
            Commands.run(() -> RobotContainer.drivetrain.alignToAngleRobotRelative(lockDrive), RobotContainer.drivetrain).onlyWhile(RobotContainer.drivetrain.inputNotIgnorable())

        );
    }
    /* aligns to angle socring mode */
    public Command alignScoringModeCommand(boolean lockDrive){
        return Commands.run(() -> RobotContainer.drivetrain.alignScoringMode(lockDrive), RobotContainer.drivetrain);
    }


    public Command alignToAngleFieldRelative(double angle, boolean lockDrive){
        return Commands.sequence(
            Commands.runOnce(() -> {RobotContainer.drivetrain.setFieldRelativeAngle(angle);}, RobotContainer.drivetrain),
            Commands.run(()->RobotContainer.drivetrain.alignToAngleFieldRelative(lockDrive)).onlyWhile(RobotContainer.drivetrain.isRobotNOTAtAngleSetPoint())
        );
    }

    public Command scoringModeTeleop(boolean lockDrive) {
        return Commands.parallel(

        //requires only sprocket
        RobotContainer.sprocketcommands.setAngleContinousCommand(RobotContainer.limelight::bestFit),

        //requires only drivetrain
        RobotContainer.drivecommands.alignScoringModeCommand(lockDrive),

        //requires only shooter
        RobotContainer.shootercommands.spinWheels(ShooterConstants.SPEAKER_SCORE_VELOCITY)
        )
        .onlyWhile(() -> RobotContainer.driverController.R2().getAsBoolean())
        .finallyDo(() -> RobotContainer.shooter.stopScoring());
    }

    public Command scoringModeAuto(boolean lockDrive) {
        return Commands.parallel(
         //requires sprocket
        RobotContainer.sprocketcommands.setAngleContinousCommand(RobotContainer.limelight::bestFit),

        //requires drivetrain
        RobotContainer.drivecommands.alignScoringModeCommand(lockDrive),

        //requires shooter
        //uses sprocket, intake, drivetrain, and shooter methods but does not
        //require the subsystem.
        RobotContainer.shootercommands.scoreSpeakerAuto()
        );
    }
}
