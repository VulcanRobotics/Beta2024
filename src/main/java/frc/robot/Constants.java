// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final String name = "Swift";

  public static final Mode currentMode = Mode.REAL;
  // public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Vision {
    public static final String kCameraName = "FrontCam";
    public static final String kCameraNameFL = "FrontLeftCam";
    public static final String kCameraNameFR = "FrontRightCam";
    public static final String kCameraNameBL = "BackLeftCam";
    public static final String kCameraNameBR = "BackRightCam";

    private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    private static final double PI = 3.1415926535;

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(0.089, -0.254, 0.3048), new Rotation3d(0, 0.349, 0));

    public static final Transform3d kRobotToCamFL =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            // new Translation3d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0, 0.3),
            new Translation3d(
                Units.inchesToMeters(10.5), Units.inchesToMeters(10.5), Units.inchesToMeters(8.0)),
            new Rotation3d(0, Math.toRadians(16), PI / 4.0));

    public static final Transform3d kRobotToCamFR =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                Units.inchesToMeters(10.5), -Units.inchesToMeters(10.5), Units.inchesToMeters(8.0)),
            new Rotation3d(0, Math.toRadians(16), -PI / 4.0));

    public static final Transform3d kRobotToCamBL =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                -Units.inchesToMeters(12.5),
                Units.inchesToMeters(12.5),
                Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(16), 3.0 * PI / 4.0));

    public static final Transform3d kRobotToCamBR =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                -Units.inchesToMeters(12.5),
                -Units.inchesToMeters(12.5),
                Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(16), -3.0 * PI / 4.0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public final class ShooterConstants {
    public static final int kGuideMotorPort = 17;
    public static final int kFollowMotorPort = 16;

    public static final int kIntakeMotor = 18;
    public static final int kFeederMotor = 19;

    public static final int kPhotogatePort = 0;

    public static final double kShooterTargetVelocity =
        95; // RPS, was 100, but feeder wouldn't spin sometimes
  }

  public final class ArmConstants {

    public static final int kGuideMotorPort = 15;
    public static final int kFollowMotorPort = 14;

    public static double kVariable = 1.0;

    public static final double kArmKP = 0.6;
    public static final double kArmKI = 0.0;
    public static final double kArmKD = 0.05;

    public static final double kArmKS = 0.25;
    public static final double kArmKV = 0.12;
    public static final double kArmKA = 0.01;

    public static final double kArmTargetVelocity = 60; // 300 Rotations per second
    public static final double kArmTargetAcceleration = 500; // rps/s
    public static final double kArmTargetJerk = 1600; // rps/s/s

    public static final boolean kArm1Inverted = false;
    public static final boolean kArm2Inverted = true;

    // Fill these with actual values (now in degrees!)
    public static final double kArmPoseAmp = 85.0;
    public static final double kArmPoseSpeaker = 0.0;
    public static final double kArmPoseIntake = 0.0;
    public static final double kArmPoseTrap = 85.0;
    public static final double kArmPoseSource = 63.8;

    public static final double kArmPIDTolerance = 0.1;

    public static final double kCanCoderToArmMotorRatio =
        (9.0 / 1.0) * (6.0 / 1.0) * (80.0 / 15.0) * (2.0 / 3.0);

    public static final double kMotorEncoderToDegrees = (5.0 / 4.0); // * kVariable; // * kVariable

    public static final double kCanCoderZeroPosition =
        0.615; // was -0.380 at cyber // this constant is cooked ngl
  }

  public static class FieldConstants {

    public static enum FieldLocations {
      SPEAKER,
      AMP,
      SOURCE,
    }

    public static Translation2d kSpeakerTargetPoseRed = new Translation2d(16.1, 5.6);
    public static Translation2d kSpeakerTargetPoseBlue = new Translation2d(0.3, 5.6);

    public static final Pose2d kSpeakerPose =
        new Pose2d(new Translation2d(1.37, 5.56), new Rotation2d(0));
    public static final Pose2d kAmpPose =
        new Pose2d(new Translation2d(1.8, 7.6), new Rotation2d(Math.PI / 2));
    public static final Pose2d kSourcePose =
        new Pose2d(new Translation2d(15.4, 1.07), new Rotation2d(-Math.PI / 4));
  }
}
