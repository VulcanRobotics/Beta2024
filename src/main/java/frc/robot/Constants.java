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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // public static final Mode currentMode = Mode.REAL;
  public static final Mode currentMode = Mode.SIM;

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
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(
            new Translation3d(0.089, -0.254, 0.3048), new Rotation3d(0, -0.349, 3.1415926535));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public final class ArmConstants {

    public static final int kGuideMotorPort = 15;
    public static final int kFollowMotorPort = 14;

    public static final double kArmKP = 0.5;
    public static final double kArmKI = 0.0;
    public static final double kArmKD = 0.05;

    public static final double kArmKS = 0.25;
    public static final double kArmKV = 0.12;
    public static final double kArmKA = 0.01;

    public static final double kArmTargetVelocity = 300; // Rotations per second
    public static final double kArmTargetAcceleration = 500; // rps/s
    public static final double kArmTargetJerk = 1600; // rps/s/s

    public static final boolean kArm1Inverted = false;
    public static final boolean kArm2Inverted = true;

    // Fill these with actual values.
    public static final double kArmPoseAmp = 76.0;
    public static final double kArmPoseSpeaker = 0.0;
    public static final double kArmPoseIntake = 0.0;
    public static final double kArmPoseTrap = 76.0;
  }
}
