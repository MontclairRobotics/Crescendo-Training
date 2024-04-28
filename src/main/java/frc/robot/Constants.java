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

    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); // TODO get real width


    public static final double MAX_VOLTAGE_V = 12.0;
    public static final double MAX_SPEED = Units.feetToMeters(13);

    public static final double MAX_ROT_SPEED = 2 * Math.PI;
    // Max Acceleration in M/s^2
    public static final double MAX_ACCELERATION = 3.0;
    // Max angular acceleration in Rad/S^2
    public static final double MAX_ANGULAR_ACCELERATION = 1.5;

    public static final double DRIVE_BASE_RADIUS = 0.43;

    public static final Pose2d EDGE_OF_DRIVEBASE =
        new Pose2d(0, DRIVE_BASE_RADIUS + BUMPER_WIDTH, new Rotation2d());
  }

  public static class VisionConstants {
    public static final double SHOOTER_LIMELIGHT_HEIGHT = 7.5; //10.375;
    public static final double INTAKE_LIMELIGHT_HEIGHT = 10.227995;
    public static final double SPEAKER_APRILTAG_HEIGHT = 57.125; //57.875;
    public static final double SPEAKER_GOAL_HEIGHT = 81.8; //78.13-82.90
    public static final double SHOOTER_LIMELIGHT_ANGLE_DEGREES = 28.4; //26.74; //31.07;
    public static final double INTAKE_LIMELIGHT_ANGLE_DEGREES = 0;

  }

  public static class Ports { // TODO: add correct ports
    public static final int LED_PWM = -1;

    public static final int INTAKE_TOP_MOTOR = -1;
    public static final int INTAKE_BOTTOM_MOTOR = -1;

    
    public static final int SHOOTER_BOTTOM_MOTOR = -1;
    public static final int SHOOTER_TOP_MOTOR = -1;
    public static final int SHOOTER_MOTOR_TRANSPORT = -1;

    // Sprocket motors
    public static final int LEFT_ANGLE_MOTOR = -1;
    public static final int RIGHT_ANGLE_MOTOR = -1;

    public static final int TRANSPORT_BEAM_BREAK = -1;

    // Climber ports
    public static final int CLIMBER_LEFT_MOTOR = -1;
    public static final int CLIMBER_RIGHT_MOTOR = -1;

    

    public static final int SPROCKET_ABS_ENCODER = 3; //9

    public static final int CLIMBER_LEFT_LIMIT_SWITCH_PORT = 5;

    public static final int CLIMBER_RIGHT_LIMIT_SWITCH_PORT = 4;
  } 

  public static class ClimberConstants {
    // Climbers
    public static final double CLIMBER_SPEED = 0.3;
    public static final double MAX_HEIGHT = 18; // Inches
    public static final double ROTATIONS_PER_INCH = Math.PI/30; //12.0672 * Math.PI; // TODO:
  }

  public static class IntakeConstants {
    
  }

  public static class ArmConstants {

    public static final double MAX_VOLTAGE_V = 12.0;
  

    
    public static final double SPROCKET_ROTATIONS_PER_DEGREE = 1.26984126984;
    public static final double ENCODER_MIN_ANGLE = 26;
    public static final double ENCODER_MAX_ANGLE = 63;

    //Both inverts are true, at least in our program (motors are mounted in same direction)
  }
}
