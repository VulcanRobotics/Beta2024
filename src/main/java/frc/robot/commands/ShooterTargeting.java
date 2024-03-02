package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

// plan to place code for autoshoot here; break it up into multiple commands (e.g. arm, shooter,
// turning)
public class ShooterTargeting {

  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelDeadlineGroup(
        /// new ShootCommand(shooter),
        new DriveToPosition(drive, drive::calculateShootingPose),
        new SetArmPosition(arm, drive::getArmShootingAngle),
        new RevCommand(shooter, false));
    /*    `
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new DriveToPosition(drive, drive::calculateShootingPose),
        new SetArmPosition(arm, drive::getArmShootingAngle)
      ).asProxy()
    );*/
  }
}
