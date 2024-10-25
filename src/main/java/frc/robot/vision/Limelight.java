package frc.robot.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase{

    public final static String llname = "limelight-shooter";
    int tagCount;
    NetworkTable Limetable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = Limetable.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    double distanceFromLimelightToSpeakerInches;
    double speakerToLimelightVerticalInches = (VisionConstants.SPEAKER_GOAL_HEIGHT 
- VisionConstants.SHOOTER_LIMELIGHT_HEIGHT);

public double getDistanceFromLimeToGoal() {
 return distanceFromLimelightToSpeakerInches;
}
public static double getTX(){
    return LimelightHelpers.getLimelightNTDouble(llname, "tx");
} 
public static double getTY() {
    return LimelightHelpers.getLimelightNTDouble(llname, "ty");
} 
public static DoubleSupplier tySupplier(){
    return () -> getTY();
}
public static DoubleSupplier txSupplier(){
    return () -> getTX();
}
public double getBotPose(){
    return LimelightHelpers.getLimelightNTDouble(llname, "botpose");
}  
public static void takeSnapshot() {
    LimelightHelpers.takeSnapshot("","Limelight Snapshot");
}
public boolean isTargetInView(){
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate(llname, "botpose");
    if(limelightMeasurement.tagCount > 0) return true;
    return false;
}
public DoubleSupplier bestFitAngleSupplier(){
    return () -> bestFit();
}
public double bestFit(){
    return bestFitFromDistance(getDistanceToSpeaker());
}
public double bestFitFromDistance(double x){
   return (59.41*Math.exp(-0.01694*x)) + 27.01;
}
public double getDistanceToSpeaker(){
    return distanceFromLimelightToSpeakerInches;
}
public void periodic(){

/*this is h2-h1/tan(a1+a2)
 * h2 is the speaker height
 * h1 is the limelight mount height
 * speakertoLimelightVerticalInches is h2-h1
 * a1 is ty
 * a2 is angle of camera
*/
distanceFromLimelightToSpeakerInches = speakerToLimelightVerticalInches
/ Math.tan(Math.toRadians(getTY() + VisionConstants.SHOOTER_LIMELIGHT_ANGLE_DEGREES));

}

    }

