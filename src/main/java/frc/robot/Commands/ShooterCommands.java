package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
   
     //fully works during teleop and autonomous, ends properly
    public Command scoreSubwoofer(){
        
        if(RobotContainer.intake.bbTriggered){
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
            .onlyWhile(RobotContainer.intake.noteOutOfTransport()))
            .andThen(Commands.runOnce(() -> System.out.println("\n\n\n SCORING SUBWOOFER RAN!!!!!!!!!\n\n\n")));
        //ending conditions
        } 
        
        
        else return Commands.parallel(
            //sets the angle to score the subwoofer
            RobotContainer.sprocketcommands.setAngleCommand(ShooterConstants.SUBWOOFER_ANGLE),
            //this sets the velocity, waits until it is at velocity, then shoots the note
            Commands.run(() -> RobotContainer.shooter.shootRPM(ShooterConstants.SPEAKER_SCORE_VELOCITY))
                .withTimeout(1)
                .finallyDo(() -> {
                    RobotContainer.shooter.stopScoring();
             })
            )
            .andThen(RobotContainer.sprocketcommands.returnToDefaultAngle()
            .onlyWhile(RobotContainer.intake.noteOutOfTransport()))
            .andThen(Commands.runOnce(() -> System.out.println("Scoring Subwoofer ran")));

    }   



        //scores speaker during auto   
        public Command scoreSpeakerAuto(){
            if(RobotContainer.intake.bbTriggered){
            return Commands.run(() -> {
                RobotContainer.shooter.spinWheels(ShooterConstants.SPEAKER_SCORE_VELOCITY, ShooterConstants.SPEAKER_SCORE_VELOCITY);
                //if shooter is at the right velocity
                if(RobotContainer.shooter.isAtVelocityRPM(
                ShooterConstants.SPEAKER_SCORE_VELOCITY, 
                ShooterConstants.SPEAKER_SCORE_VELOCITY)
                //and if the sprocket is aligned
                && RobotContainer.sprocket.isAtAngle()
                //and if the drivetrain is aligned
                && RobotContainer.drivetrain.isAlignedTX()
                //it will run the transport
                ) Transport.start();
            }, RobotContainer.shooter)
            .until(RobotContainer.intake.noteOutOfTransport())
            .finallyDo(() -> RobotContainer.shooter.stopScoring());
        } else return Commands.runOnce(() -> {});
        } 



        //scores speaker teleop SCORING MODE ONLY
        public Command scoreSpeakerTeleop(){
            return Commands.run(() -> Transport.start())
            .withTimeout(0.2)
            .finallyDo(() -> RobotContainer.shooter.stopScoring());
        }

        public BooleanSupplier getR2(){
            return () -> RobotContainer.driverController.R2().getAsBoolean();
        }

        //does not work, crashes the robot when used during scoring mode

        //ideass to fix
        //make getR2() always return true or false and see what happens
        //replace scoreSpeakerTeleop with like a command that waits for two seconds
        //test scoreSpeakerTeleop without scoringModeTeleop running

        public Command scoreSpeakerDecider(){
            ConditionalCommand c = new ConditionalCommand(scoreSpeakerTeleop(), scoreSubwoofer(), getR2());
            return c;
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
