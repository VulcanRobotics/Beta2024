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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double AnglePIDValues[] = {0.45, 0.0, 0.0};
  private static final double NoteTrackPIDValues[] = {0.005, 0.0, 0.0};
  private static final double TranslationPIDValues[] = {1.2, 0.0, 0.01};
  private static final PIDController xTranslationController =
      new PIDController(TranslationPIDValues[0], TranslationPIDValues[1], TranslationPIDValues[2]);
  private static final PIDController yTranslationController =
      new PIDController(TranslationPIDValues[0], TranslationPIDValues[1], TranslationPIDValues[2]);
  private static final PIDController angleController =
      new PIDController(AnglePIDValues[0], AnglePIDValues[1], AnglePIDValues[2]);
  private static final PIDController noteTrackController =
      new PIDController(NoteTrackPIDValues[0], NoteTrackPIDValues[1], NoteTrackPIDValues[2]);

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          var alliance = DriverStation.getAlliance();
          var invert = 1;
          if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
          }
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Allows the driver to drive while aiming at a target (the speaker, for example). */
  public static Command driveWhileAiming(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose2d> thetaSupplier) {

    angleController.reset();
    angleController.setTolerance(Math.toRadians(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          Rotation2d targetTheta = thetaSupplier.get().getRotation();

          var omega =
              angleController.calculate(
                  robotPose.getRotation().getRadians(),
                  targetTheta.getRadians()
                      + (DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? Math.PI
                          : 0.0));

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Command that auto-aligns with the amp and prepares to score. Most useful if the driver does not
   * have good amp visibility.
   */
  public static Command driveToAmp(Drive drive, Supplier<Pose2d> poseSupplier) {

    Translation2d amp;

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      amp = Constants.FieldConstants.kAmpTargetPoseRed;
    } else {
      amp = Constants.FieldConstants.kAmpTargetPoseBlue;
    }

    angleController.reset();
    angleController.setTolerance(Math.toRadians(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    yTranslationController.reset();
    xTranslationController.reset();
    yTranslationController.setTolerance(0.05);
    xTranslationController.setTolerance(0.05);
    yTranslationController.setSetpoint(amp.getY());
    xTranslationController.setSetpoint(amp.getX());

    return Commands.run(
        () -> {
          Pose2d pose = poseSupplier.get();
          double x = poseSupplier.get().getX();
          double y = poseSupplier.get().getY();
          double xSpeed = xTranslationController.calculate(x);
          double ySpeed = yTranslationController.calculate(y);

          xSpeed = MathUtil.clamp(xSpeed, -0.8, 0.8);
          ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);

          double thetaSpeed =
              angleController.calculate(
                  pose.getRotation().getRadians(),
                  (-Math.PI / 2)
                      + (DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? Math.PI
                          : 0.0));

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  thetaSpeed * drive.getMaxAngularSpeedRadPerSec(),
                  (isFlipped)
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Allows the driver to use ML to turn to a note and intake it. */
  public static Command driveWhileNoteTracking(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    noteTrackController.reset();
    noteTrackController.setTolerance(Math.toRadians(0.25));
    // noteTrackController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          Optional<Double> tx = LimelightVisionSubsystem.getNoteDistLL();
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          var omega =
              (tx.isPresent())
                  ? noteTrackController.calculate(tx.get(), 0.0)
                  : MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
