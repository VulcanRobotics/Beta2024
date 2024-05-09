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

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);
  private static final double shotSpeed = 5.0; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  public static Command shoot(Drive drive, ArmSubsystem arm) {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  double x = 0.64 * Math.cos(Math.PI * 2 * arm.getArmEncoder() / 360);
                  double z = 0.64 * Math.sin(Math.PI * 2 * arm.getArmEncoder() / 360);

                  final Pose3d startPose =
                      new Pose3d(drive.getPose())
                          .transformBy(
                              new Transform3d(
                                  x, 0, z, new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0)));
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }
}
