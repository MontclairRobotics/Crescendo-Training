package frc.robot.Commands;

// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.RobotContainer;
// import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.Shooter;
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
             })
            )
            .andThen(RobotContainer.sprocketcommands.returnToDefaultAngle()
            .onlyWhile(RobotContainer.intake.noteOutOfTransport()));
        
        //ending conditions
        
    }   

        //scores speaker during auto   
        public Command scoreSpeakerAuto(){
            return Commands.run(() -> {
                RobotContainer.shooter.spinWheels(ShooterConstants.SPEAKER_SCORE_VELOCITY, ShooterConstants.SPEAKER_SCORE_VELOCITY);
                if(RobotContainer.shooter.isAtVelocityRPM(ShooterConstants.SPEAKER_SCORE_VELOCITY, ShooterConstants.SPEAKER_SCORE_VELOCITY)
                    && RobotContainer.sprocket.isAtAngle()
                    && RobotContainer.drivetrain.isAlignedTX()){
                Transport.start();
                }
            }, RobotContainer.shooter)
            .withTimeout(0.5)
            .finallyDo(() -> RobotContainer.shooter.stopScoring());
        } 

        //scores speaker teleop 
        public Command scoreSpeakerTeleop(){
            return Commands.run(() -> Transport.start())
            .withTimeout(0.2)
            .finallyDo(() -> RobotContainer.shooter.stopScoring());
        }

        public Command scoreSpeakerDecider(){
            if(RobotContainer.shooter.scoringMode) return scoreSpeakerTeleop();
            else return scoreSubwoofer();
        }
  
  
    /*
     * 
     * SCORE AMP
     * 
     */

    public Command scoreAmp(){
        return Commands.parallel(
            Commands.run(() -> RobotContainer.shooter.spinWheels(ShooterConstants.AMP_TOP_SPEED, ShooterConstants.AMP_BOTTOM_SPEED), RobotContainer.shooter),
            
            RobotContainer.sprocketcommands.setAngleCommand(ShooterConstants.AMP_SCORE_ANGLE),
            
            Commands.waitUntil(RobotContainer.sprocket.isAtAngleSupplier())
            .andThen(Commands.run(() -> RobotContainer.shooter.shootRPM(ShooterConstants.AMP_TOP_SPEED, ShooterConstants.AMP_BOTTOM_SPEED))
)
        )
        .withTimeout(.7)
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
