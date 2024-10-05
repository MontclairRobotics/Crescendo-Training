package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase{

    public final String llname = "shooter-limelight";
    int tagCount;
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
    return LimelightHelpers.getLimelightNTDouble(llname, "tx");
} 
public double getTY() {
    return LimelightHelpers.getLimelightNTDouble(llname, "ty");
} 
public double getBotPose(){
    return LimelightHelpers.getLimelightNTDouble(llname, "botpose");
}  
public static void takeSnapshot() {
    LimelightHelpers.takeSnapshot("","Limelight Snapshot");
}
public boolean isTargetInView(){
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate(llname, "entry");
    if(limelightMeasurement.tagCount > 0) return true;
    return false;
}

public void setPipeline(Pipetype pipe) {
    if(pipe == Pipetype.DRIVER){ 
        LimelightHelpers.setCameraMode_Driver(llname);
    }
    if(pipe == Pipetype.NOTE){
        LimelightHelpers.setCameraMode_Processor(llname);
    }
}
    }

