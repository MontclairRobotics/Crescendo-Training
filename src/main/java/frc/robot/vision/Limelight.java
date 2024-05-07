package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Limelight {
static String camera;
NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry ty = Limetable.getEntry("ty");
//TODO: Change Values
double targetOffsetAngle_Vertical = ty.getDouble(0.0);
double distanceFromLimelightToGoalInches = (Constants.VisionConstants.SPEAKER_GOAL_HEIGHT - Constants.VisionConstants.SHOOTER_LIMELIGHT_HEIGHT) / Math.tan(Constants.VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES);
  
public double getDistanceFromLimetoGoal() {
 return distanceFromLimelightToGoalInches;
}
public void addVisionMeasurement(Pose2d robotPose, double timestamp){

}

public double getTX(){
    return LimelightHelpers.getLimelightNTDouble(camera, "tx");
} 
public double getTY(){
    return LimelightHelpers.getLimelightNTDouble(camera, "ty");
}   
    public static void takeSnapshot() {
        LimelightHelpers.takeSnapshot("","Limelight Snapshot");
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
        return Commands.runOnce(()-> Limelight.takeSnapshot());
    }
}
