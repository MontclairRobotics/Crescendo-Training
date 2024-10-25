package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sprocket;

public class IntakeCommands extends Command {
    /*
     * 
     * 
     * ALL OF THE INTAKING RELATED COMMANDS BELOW ->
     * 
     * 
     */
    
    public Command intake(){
        return Commands.parallel(
            //sets the sprocket to the intake angle
            RobotContainer.sprocketcommands.setAngleCommand(ArmConstants.SPROCKET_INTAKE_ANGLE),
            //runs the intake until there is a note in the transport
            Commands.run(() -> RobotContainer.intake.start(), RobotContainer.intake)
            .onlyWhile(RobotContainer.getL2())
            .until(RobotContainer.intake.noteInTransport())
            .finallyDo(() -> RobotContainer.intake.stop())
        );
    }

    public Command outtake(){
        return Commands.parallel(
            //sets the sprocket to the outtake angel
            RobotContainer.sprocketcommands.setAngleCommand(ArmConstants.SPROCKET_OUTTAKE_ANGLE),
            //runs the outtake
            Commands.run(() -> RobotContainer.intake.reverse(), RobotContainer.intake)
            .onlyWhile(RobotContainer.getR2())
            .finallyDo(() -> RobotContainer.intake.stop())
        );
    }

    public Command stopIntake(){
        return Commands.runOnce(() -> {
            RobotContainer.intake.stop();
            RobotContainer.shooter.stop();
        });
    }

    public Command intakeSourceCommand(){
    return Commands.sequence(

        /* SETS THE SPROCKET ANGLE */
        RobotContainer.sprocketcommands.setAngleCommand(ArmConstants.SOURCE_INTAKE_ANGLE),

        /* THEN SPINS UP THE SHOOTER UNTIL BEAMBREAK TRIGGERS */
        Commands.run(() -> RobotContainer.shooter.intakeSource())
        .until(RobotContainer.intake.noteInTransport()),

       
        /* 
         * CONTINUES TO SPIN UP SHOOTER UNTIL THE NOTE EXITS THE BEAMBREAK
         * THE NOTE WILL NOW BE IN THE TRANSPORT
         */
        Commands.run(()-> RobotContainer.shooter.intakeSource())
        .until(RobotContainer.intake.noteOutOfTransport())
        .finallyDo(() -> {
            RobotContainer.intake.stop();
            RobotContainer.shooter.stop();
        })
        );
    }
}
