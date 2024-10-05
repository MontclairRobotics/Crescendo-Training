package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LimelightJames {
    static String cameraName = "shooter Limelight";
    NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = Limetable.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double distanceFromLimelightToGoalInches = (Constants.VisionConstants.SPEAKER_GOAL_HEIGHT - Constants.VisionConstants.SHOOTER_LIMELIGHT_HEIGHT) / Math.tan(Constants.VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES);

public double getDistanceFromLimetoGoal() {
 return distanceFromLimelightToGoalInches;
}
public double getTX(){
    return LimelightHelpers.getLimelightNTDouble(cameraName, "tx");
} 
public double getTY() {
    return LimelightHelpers.getLimelightNTDouble(cameraName, "ty");
} 
public double getBotPose(){
    return LimelightHelpers.getLimelightNTDouble(cameraName, "botpose");
}  
public static void takeSnapshot() {
    LimelightHelpers.takeSnapshot("","Limelight Snapshot");
}

public void setDefaultPipe() {
    LimelightHelpers.setCameraMode_Driver(cameraName);
}
public static void estimatePose() {
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(limelightMeasurement.tagCount >= 2)
    {
        RobotContainer.drivetrain.swerveDrive.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
    }
}

    public Command takeSnapshotCommand() {
        return Commands.runOnce(()-> LimelightJames.takeSnapshot());
    }
}
