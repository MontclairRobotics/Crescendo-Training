package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.IntakeConstants;

public class SprocketCommands extends Command {

    /*
     * 
     * 
     * 
     * ALL SPROCKET RELATED COMMANDS FOUND HERE ->
     * 
     * 
     * 
     */


    /* SETS THE ANGLE ONCE */
    public Command setAngleCommand(double angle) {
        return Commands.runOnce(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(angle)), RobotContainer.sprocket);
    }

    /* SETS THE ANGLE CONTINOUSLY */
    /*
     * Pass in a double supplier and it will continuosly align to that angle even if it is changing
     * used for scoring mode
     */
    public Command setAngleContinousCommand(DoubleSupplier angDegSupplier){
        return Commands.run(() -> RobotContainer.sprocket.setAngle(Rotation2d.fromDegrees(angDegSupplier.getAsDouble())), RobotContainer.sprocket);
    }

    /* COMMAND TO STOP THE SPROCKET */
    public Command stop(){
        return Commands.runOnce(()-> RobotContainer.sprocket.stop());
    }

    /* SETS BRAKE MODE */
    public Command setBrakeModeCommand(){
        return Commands.run(()-> RobotContainer.sprocket.setBrakeMode(), RobotContainer.sprocket);
    }

    /* SETS COAST MODE */
    public Command setCoastModeCommand(){
        return Commands.runOnce(()-> RobotContainer.sprocket.setCoastMode(), RobotContainer.sprocket);
    }

    /* DEFAULT COMMAND */
    /*
     * Manual input from joysticks to move sprocket up and down
     * 
     */
    public Command sprocketDefaultCommand(){
        return Commands.run(()-> RobotContainer.sprocket.sprocketDefault(), RobotContainer.sprocket);
    }

    /* RETURNS TO THE INTAKE ANGLE, WHICH IS THE DEFAULT ANGLE */
    public Command returnToDefaultAngle(){
        return RobotContainer.sprocketcommands.setAngleCommand(ArmConstants.SPROCKET_INTAKE_ANGLE);
    }
}
