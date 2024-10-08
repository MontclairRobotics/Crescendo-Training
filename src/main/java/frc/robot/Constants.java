// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import animation2.AnimationReel;
import animation2.CircusAnimation;
import animation2.MagicAnimation;
import animation2.QuickSlowFlash;
import animation2.RaceAnimation;
import animation2.RainbowAnimation;
import animation2.WipeTransition;
import animation2.api.Animation;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Tunable;
import java.awt.geom.Point2D;


public final class Constants {

  public static class FieldConstants {

    //The poses we determined to be best for scoring using odometry. These were not used in our program, feel free to change them.
    public static final double SPEAKER_SCORE_X_OFFSET = 6.0;
    
    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.3 - SPEAKER_SCORE_X_OFFSET), Units.inchesToMeters(218.42), new Rotation2d());
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.5 + SPEAKER_SCORE_X_OFFSET), Units.inchesToMeters(218.42), new Rotation2d());
    
  }

  public static class DriveConstants {

    //todo: get real width
    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); 

    //max voltage of the battery
    public static final double MAX_VOLTAGE_V = 12.0;

    //max speed of the robot
    public static final double MAX_SPEED = Units.feetToMeters(13);

    //this is in radians. for those who don't know, 2pi radians = 360 degrees
    //our max rotation speed is therefore 360degrees/s or 2pi radians/second
    public static final double MAX_ROT_SPEED = 2 * Math.PI;

    // Max Acceleration in M/s^2
    public static final double MAX_ACCELERATION = 3.0;

    // Max angular acceleration in Rad/S^2
    public static final double MAX_ANGULAR_ACCELERATION = 1.5;

    //distance from the center of the robot to the pivot point on the swerve modules in METERS
    //used in autoBuilder
    public static final double DRIVE_BASE_RADIUS = 0.43;

    public static final Pose2d EDGE_OF_DRIVEBASE = new Pose2d(0, DRIVE_BASE_RADIUS + BUMPER_WIDTH, new Rotation2d());
  
  }

  public static class VisionConstants {

    public static final double SHOOTER_LIMELIGHT_HEIGHT = 7.5; //10.375;
    public static final double INTAKE_LIMELIGHT_HEIGHT = 10.227995;
    public static final double SPEAKER_APRILTAG_HEIGHT = 57.125; //57.875;
    public static final double SPEAKER_GOAL_HEIGHT = 81.8; //78.13-82.90
    public static final double SHOOTER_LIMELIGHT_ANGLE_DEGREES = 28.4; //26.74; //31.07;
    public static final double INTAKE_LIMELIGHT_ANGLE_DEGREES = 0;

  }

  public static class Ports { // todo: add correct ports (NEGATIVE PORTS CRASH THE CODE SO CHANGE BEFORE USING PLS)

    //don't know what this is
    public static final int LED_PWM = -1;

    //intake motors
    public static final int INTAKE_TOP_MOTOR = 21;
    public static final int INTAKE_BOTTOM_MOTOR = 20;
    
    // shooter motors
    public static final int SHOOTER_BOTTOM_MOTOR = 29;
    public static final int SHOOTER_TOP_MOTOR = 28;
    public static final int SHOOTER_MOTOR_TRANSPORT = 27;

    // Sprocket motors
    public static final int LEFT_ANGLE_MOTOR = 30;
    public static final int RIGHT_ANGLE_MOTOR = 31;

    //where the beambreak plugs into the roboRIO (this is how you access it)
    public static final int TRANSPORT_BEAM_BREAK = 9;

    // Climber ports
    public static final int CLIMBER_LEFT_MOTOR = 48;
    public static final int CLIMBER_RIGHT_MOTOR = 49;
    public static final int CLIMBER_LEFT_LIMIT_SWITCH_PORT = 5;
    public static final int CLIMBER_RIGHT_LIMIT_SWITCH_PORT = 4;

    // where arm encoder plugs in
    public static final int SPROCKET_ABS_ENCODER = 2; //told to me by taiki and anthony on 10/2/2024
  } 

  public static class ClimberConstants {

    // Climbers
    public static final double CLIMBER_SPEED = 0.3;
    public static final double MAX_HEIGHT = 18; // Inches
    public static final double ROTATIONS_PER_INCH = Math.PI/30; //12.0672 * Math.PI;

  }

  public static class IntakeConstants {

  //self-explanatory guys...
  public static final double INTAKE_SPEED = -.7;

  }

  //transport speed
  public static final double TRANSPORT_SPEED = -.7;

  //arm stuff
  public static class ArmConstants {
    public static final double SPROCKET_ROTATIONS_PER_DEGREE = 1.26984126984;
    public static final double ENCODER_MIN_ANGLE = 26;
    public static final double ENCODER_MAX_ANGLE = 63;
    public static final double SPROCKET_SPEED = .5;
    public static final double SPROCKET_ANGLE_DEADBAND = 2;
    public static final double SPROCKET_OFFSET = -49.2 + 1;
    public static final double SPROCKET_INTAKE_ANGLE = 40;
    public static final double SPROCKET_OUTTAKE_ANGLE = 35;

    //Both inverts are true, at least in our program (motors are mounted in same direction)
  }
  //Shooter constants
  public class ShooterConstants {
      public static final double SPEAKER_SPEED_RPS = 25;
      public static final double SHOOT_SPEAKER_VELOCITY = 4000;
      public static final double AMP_TOP_SPEED = 600;
      public static final double AMP_BOTTOM_SPEED = 800;
      public static final double AMP_SCORE_ANGLE = 60;
      public static final double AMP_TOP_RPS = 10;
      public static final double AMP_BOTTOM_RPS = 13.33333;
      public static final double SHOOTER_VELOCITY_DEADBAND_RPM = 60;
      public static final double SHOOTER_VELOCITY_DEADBAND_RPS = 1;
      

        // From SysId
    // public static final double TOP_SHOOTER_FF_KS = 0.20823;
    // public static final double TOP_SHOOTER_FF_KV = 0.0021625;
    // public static final double TOP_SHOOTER_FF_KA = 0.00024932;
    // public static final double TOP_SHOOTER_PID_KP = 3.8326e-07;
    // public static final double TOP_SHOOTER_PID_KI = 0;
    // public static final double TOP_SHOOTER_PID_KD = 0;
    public static final double TOP_SHOOTER_FF_KS = 0.25244; // 0.18454
    public static final double TOP_SHOOTER_FF_KV = 0.0022082; // 0.0021629
    public static final double TOP_SHOOTER_FF_KA = 0.0002064; // 0.00026348
    public static final double TOP_SHOOTER_PID_KP = 1.932E-09; //4.4848e-07
    public static final double TOP_SHOOTER_PID_KI = 0;
    public static final double TOP_SHOOTER_PID_KD = 0;

    // public static final double BOTTOM_SHOOTER_FF_KS = 0.22812;
    // public static final double BOTTOM_SHOOTER_FF_KV = 0.0022851;
    // public static final double BOTTOM_SHOOTER_FF_KA = 0.00026402;
    // public static final double BOTTOM_SHOOTER_PID_KP = 4.1584e-07;
    // public static final double BOTTOM_SHOOTER_PID_KI = 0;
    // public static final double BOTTOM_SHOOTER_PID_KD = 0;
    public static final double BOTTOM_SHOOTER_FF_KS = 0.22311; // 0.19665
    public static final double BOTTOM_SHOOTER_FF_KV = 0.0021558; // 0.0022763
    public static final double BOTTOM_SHOOTER_FF_KA = 0.00021675; // 0.00033793 
    public static final double BOTTOM_SHOOTER_PID_KP = 8.0536E-09; // 7.1995e-07
    public static final double BOTTOM_SHOOTER_PID_KI = 0;
    public static final double BOTTOM_SHOOTER_PID_KD = 0;

    public static final double TRANSPORT_FF_KS = 0.15883;
    public static final double TRANSPORT_FF_KV = 0.0020936;
    public static final double TRANSPORT_FF_KA = 0.00017895;
    public static final double TRANSPORT_PID_KP = 1.6005e-07;
    public static final double TRANSPORT_PID_KI = 0;
    public static final double TRANSPORT_PID_KD = 0;
  }
  
}
