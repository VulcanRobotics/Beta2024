package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

// plan to place code for autoshoot here; break it up into multiple commands (e.g. arm, shooter,
// turning)
public class ShooterTargeting {

  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelCommandGroup(
        new DriveToPosition(
            drive,
            new Pose2d(
                new Translation2d(drive.getPose().getX(), drive.getPose().getY()),
                new Rotation2d(0.0))),
        new SetArmPosition(arm, 0),
        new RevCommand(shooter, false),
        new ShootCommand(shooter));
  }

  public static Rotation2d calculateShootingAngle(Drive drive) {

    return new Rotation2d(0);
  }
}
