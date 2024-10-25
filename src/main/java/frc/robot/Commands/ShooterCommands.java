package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;

public class ShooterCommands extends Command {
    /*
     * 
     * 
     * ALL OF THE SHOOTER (RELATED) COMMANDS HERE ->
     * 
     * 
     */

    /*
     * Command sequence to score the subwoofer
     */
   
    public Command scoreSubwoofer(){
        return Commands.parallel(
            //sets the angle to score the subwoofer
            RobotContainer.sprocketcommands.setAngleCommand(ShooterConstants.SUBWOOFER_ANGLE),
            //this sets the velocity, waits until it is at velocity, then shoots the note
            Commands.run(() -> RobotContainer.shooter.shootRPM(ShooterConstants.SPEAKER_SCORE_VELOCITY))
                .until(RobotContainer.intake.noteOutOfTransport())
                .finallyDo(() -> {
                    RobotContainer.shooter.stopScoring();
                    RobotContainer.sprocketcommands.returnToDefaultAngle();
             }),
             RobotContainer.sprocketcommands.returnToDefaultAngle()
                .onlyWhile(RobotContainer.intake.noteOutOfTransport())
            );
        
        //ending conditions
        
        }       

    public Command scoreSpeaker(boolean duringAuto){

        /*
         * FIRST OPTION
         * 
         * this runs if we are running auto and scoring mode
         * it will wait for shooter to be at velocity and then run the transport, then end 
         */

        if(duringAuto){
            return Commands.run(() -> {
                if(RobotContainer.shooter.isAtVelocityRPM(ShooterConstants.SPEAKER_SCORE_VELOCITY, ShooterConstants.SPEAKER_SCORE_VELOCITY)){
                Transport.start();
                }
            })
            //ending conditions
            .onlyWhile(RobotContainer.shooter.noteReadyToShoot())
            .finallyDo(() -> {
                RobotContainer.shooter.stopScoring();
            })
            .andThen(RobotContainer.sprocketcommands.returnToDefaultAngle());

            /* SECOND OPTION

             * this will run if scoring mode is not during auto and operator is pressing circle
            */

        } else if(RobotContainer.shooter.scoringMode && RobotContainer.operatorController.circle().getAsBoolean()){
            return Commands.run(() -> {
                Transport.start();
            })
            .onlyWhile(RobotContainer.shooter.noteReadyToShoot())
            .finallyDo(() -> {
                RobotContainer.shooter.stopScoring();
            })
            .andThen(RobotContainer.sprocketcommands.returnToDefaultAngle());
        }

        /*
         * LAST OPTION
         * if those don't run than score subwoofer
         */
        
        else {
            return scoreSubwoofer();
        }
    }
  
  
    /*
     * 
     * SCORE AMP
     * 
     */

    public Command scoreAmp(){
        return Commands.sequence(
            RobotContainer.sprocketcommands.setAngleCommand(ShooterConstants.AMP_SCORE_ANGLE),
            Commands.waitUntil(RobotContainer.sprocket.isAtAngle()),
            Commands.run(() -> RobotContainer.shooter.shootRPM(ShooterConstants.AMP_TOP_SPEED, ShooterConstants.AMP_BOTTOM_SPEED))
        )
        .until(RobotContainer.intake.noteOutOfTransport())
        .finallyDo(() -> RobotContainer.shooter.stopScoring())
        .andThen(RobotContainer.sprocketcommands.returnToDefaultAngle());
    }

    public Command stop(){
        return Commands.runOnce(() -> RobotContainer.shooter.stop());
      }

      /* SPINS WHEELS BUT NOT TRANSPORT */
    public Command spinWheels(double vel){
        return Commands.run(() -> RobotContainer.shooter.spinWheels(vel, vel), RobotContainer.shooter);
    }


    
}
