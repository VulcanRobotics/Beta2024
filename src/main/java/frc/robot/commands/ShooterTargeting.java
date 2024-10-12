package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

/** This class contains command compositions for different shooting functions */
public class ShooterTargeting {

  /** Aims at the speaker and shoots the note, stops and continues if there is no note present */
  public static Command shootAtTarget(Drive drive) {
    return new SequentialCommandGroup(new DriveToPosition(drive, drive::calculateShootingPose))
        .withTimeout(2.0);
  }

  /** Used to pass notes from the center line to the amp/speaker area on our side of the field. */
  public static Command shuttleShoot(Drive drive) {
    return new ParallelCommandGroup(new DriveToPosition(drive, drive::calculateShuttlePose));
  }
}
