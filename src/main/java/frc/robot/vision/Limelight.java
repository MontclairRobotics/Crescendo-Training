package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase{

    public final String llName = "shooter-limelight";
    NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = Limetable.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double distanceFromLimelightToGoalInches = (Constants.VisionConstants.SPEAKER_GOAL_HEIGHT 
    - Constants.VisionConstants.SHOOTER_LIMELIGHT_HEIGHT) / Math.tan
    (Constants.VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES);

public double getDistanceFromLimeToGoal() {
 return distanceFromLimelightToGoalInches;
}
public double getTX(){
    return LimelightHelpers.getLimelightNTDouble(llName, "tx");
} 
public double getTY() {
    return LimelightHelpers.getLimelightNTDouble(llName, "ty");
} 
public double getBotPose(){
    return LimelightHelpers.getLimelightNTDouble(llName, "botpose");
}  
public static void takeSnapshot() {
    LimelightHelpers.takeSnapshot("","Limelight Snapshot");
}
    }

