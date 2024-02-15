package frc.robot.subsystems;

import java.io.File;


// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import static edu.wpi.first.units.Units.*;

public class Drivetrain extends SubsystemBase {
    
    public final SwerveDrive swerveDrive;

    // @AutoLogOutput
    private boolean isFieldRelative;
    // private AHRS navX;
    
    public Drivetrain(File directory) {

        this.isFieldRelative = true;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }

        // PathPlannerLogging.setLogActivePathCallback(
        //     (activePath) -> {
        //       Logger.recordOutput(
        //           "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        //     });
        // PathPlannerLogging.setLogTargetPoseCallback(
        //     (targetPose) -> {
        //       Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        //     });
        

    }
    /**
     * It drives a certain distance with a certain rotation
     */
    public void drive(Translation2d translation, double rotation) {

        swerveDrive.drive(translation, rotation, this.isFieldRelative, true);

    }
    /**
     * Moves chassis
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }
    /**
     * logs data: module positions, gyro rotation, and pose
     */
    @Override
    public void periodic() {
        // Logger.recordOutput("Drivetrain/Module-Positions",getSwerveDrive().getModulePositions());
        // Logger.recordOutput("Drivetrain/Gyro-Rotation",getSwerveDrive().getGyroRotation3d());
        // Logger.recordOutput("Drivetrain/Pose",getSwerveDrive().getPose());        
        RobotContainer.field.setRobotPose(swerveDrive.getPose());
    }


    public SysIdRoutine getSysIdAngle() {

        MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
        MutableMeasure<Angle> rotations = MutableMeasure.mutable(Rotations.of(0));
        MutableMeasure<Velocity<Angle>> motorVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));

        SwerveModule[] modules = swerveDrive.getModules();
        CANSparkMax frontLeft = (CANSparkMax) modules[0].getAngleMotor().getMotor();
        CANSparkMax frontRight = (CANSparkMax) modules[1].getAngleMotor().getMotor();
        CANSparkMax backLeft = (CANSparkMax) modules[2].getAngleMotor().getMotor();
        CANSparkMax backRight = (CANSparkMax) modules[3].getAngleMotor().getMotor();

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
                backLeft.setVoltage(volts.in(Volts));
                backRight.setVoltage(volts.in(Volts));
            }, 
            (SysIdRoutineLog log) -> {
                log.motor("front-left-steer")
                .voltage(appliedVoltage.mut_replace(frontLeft.getAppliedOutput() * frontLeft.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(frontLeft.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(frontLeft.getEncoder().getPosition(), Rotations));

                log.motor("front-right-steer")
                .voltage(appliedVoltage.mut_replace(frontRight.getAppliedOutput() * frontRight.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(frontRight.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(frontRight.getEncoder().getPosition(), Rotations));

                log.motor("back-left-steer")
                .voltage(appliedVoltage.mut_replace(backLeft.getAppliedOutput() * backLeft.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(backLeft.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(backLeft.getEncoder().getPosition(), Rotations));

                log.motor("back-right-steer")
                .voltage(appliedVoltage.mut_replace(backRight.getAppliedOutput() * backRight.getBusVoltage(), Volts))
                .angularVelocity(motorVelocity.mut_replace(backRight.getEncoder().getVelocity(), RotationsPerSecond))
                .angularPosition(rotations.mut_replace(backRight.getEncoder().getPosition(), Rotations));
            },
            this
            )
        );

        return routine;
    }

    public SysIdRoutine getSysIdDrive() {
        MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
        MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
        MutableMeasure<Velocity<Distance>> linearVelocity = MutableMeasure.mutable(MetersPerSecond.of(0));


        SwerveModule[] modules = swerveDrive.getModules();
        CANSparkMax frontLeft = (CANSparkMax) modules[0].getDriveMotor().getMotor();
        CANSparkMax frontRight = (CANSparkMax) modules[1].getDriveMotor().getMotor();
        CANSparkMax backLeft = (CANSparkMax) modules[2].getDriveMotor().getMotor();
        CANSparkMax backRight = (CANSparkMax) modules[3].getDriveMotor().getMotor();

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
                backLeft.setVoltage(volts.in(Volts));
                backRight.setVoltage(volts.in(Volts));
            }, 
            (SysIdRoutineLog log) -> {
                log.motor("front-left-drive")
                .voltage(appliedVoltage.mut_replace(frontLeft.getAppliedOutput() * frontLeft.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(frontLeft.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(frontLeft.getEncoder().getPosition() * 1 /* TODO ticks to meters */, Meters));

                log.motor("front-right-drive")
                .voltage(appliedVoltage.mut_replace(frontRight.getAppliedOutput() * frontRight.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(frontRight.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(frontRight.getEncoder().getPosition(), Meters));

                log.motor("back-left-drive")
                .voltage(appliedVoltage.mut_replace(backLeft.getAppliedOutput() * backLeft.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(backLeft.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(backLeft.getEncoder().getPosition(), Meters));

                log.motor("back-right-drive")
                .voltage(appliedVoltage.mut_replace(backRight.getAppliedOutput() * backRight.getBusVoltage(), Volts))
                .linearVelocity(linearVelocity.mut_replace(backRight.getEncoder().getVelocity(), MetersPerSecond))
                .linearPosition(distance.mut_replace(backRight.getEncoder().getPosition(), Meters));
            },
            this
            )
        );
        
        return routine;
    }

    

    public void setDriveVoltage(Measure<Voltage> volts) {
        SwerveModule[] modules = swerveDrive.getModules();
        for (SwerveModule module : modules) {
            module.getDriveMotor().setVoltage(volts.in(Volts));
        }
    }

    public void driveWithAngleVoltage(Measure<Voltage> volts) {
        SwerveModule[] modules = swerveDrive.getModules();
        for (SwerveModule module : modules) {
            module.getAngleMotor().setVoltage(volts.in(Volts));
        }
    }

    /**
     * sets isFieldRelative to either true or false, used for getIsFieldRelative
     */
    public void setIsFieldRelative(boolean relative) {
        this.isFieldRelative = relative;
    }
    /**
     * Returns if the paramter 
     */
    public boolean getIsFieldRelative(boolean relative) {
        return this.isFieldRelative;
    }

    public void zeroGyro() {
        this.swerveDrive.zeroGyro();
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }
    /**
     * Resets the odometer 
     */
    public void resetOdometry() {
        this.swerveDrive.resetOdometry(new Pose2d(0.0,0.0, new Rotation2d(0.0)));
    }
    // @AutoLogOutput
    /**
     * returns direction
     */
    public Rotation2d getRotation() {
        
        return this.swerveDrive.getOdometryHeading();
    }
    
  
    public void setInputFromController(CommandPS5Controller controller) {
      
        double thetaSpeed = MathUtil.applyDeadband(controller.getRightX(), 0.05) * DriveConstants.MAX_ROT_SPEED;

        double xSpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.05) * DriveConstants.MAX_SPEED;
        double ySpeed = MathUtil.applyDeadband(controller.getLeftY(), 0.05) * DriveConstants.MAX_SPEED;
        
        Translation2d targetTranslation = new Translation2d(ySpeed,xSpeed);
        // Logger.recordOutput("Drivetrain/Controller-Translation", targetTranslation);
        // Logger.recordOutput("Drivetrain/Controller-Theta", thetaSpeed);

        this.drive(targetTranslation, thetaSpeed);
    }
    

}




