package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.io.File;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drivetrain extends SubsystemBase {
    
    private final SwerveDrive swerveDrive;
    private boolean isFieldRelative;
    
    public Drivetrain(File directory, boolean isFieldRelative) {

        this.isFieldRelative = isFieldRelative;

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED,2, 2);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }

    }

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, this.isFieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void setIsFieldRelative(boolean relative) {
        this.isFieldRelative = relative;
    }
    public boolean getIsFieldRelative(boolean relative) {
        return this.isFieldRelative;
    }
}
