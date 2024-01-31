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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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

  public final class ArmConstants {

    public static final double kArmKP = 0.5;
    public static final double kArmKI = 0.0;
    public static final double kArmKD = 0.1;

    public static final double kArmKS = 0.25;
    public static final double kArmKV = 0.12;
    public static final double kArmKA = 0.01;

    public static final double kArmTargetVelocity = 80; // Rotations per second
    public static final double kArmTargetAcceleration = 200; // rps/s
    public static final double kArmTargetJerk = 1600; // rps/s/s

    public static final boolean kArm1Inverted = false;
    public static final boolean kArm2Inverted = true;

    // Fill these with actual values.
    public static final double kArmPoseAmp = 0.0;
    public static final double kArmPoseSpeaker = 0.0;
    public static final double kArmPoseIntake = 0.0;
    public static final double kArmPoseTrap = 0.0;

    public static enum ArmMode {
      AMP,
      SPEAKER,
      INTAKE,
      TRAP
    }
  }
}
