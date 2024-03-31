package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

// plan to place code for autoshoot here; break it up into multiple commands (e.g. arm, shooter,
// turning)
public class ShooterTargeting {

  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new DriveToPosition(drive, drive::calculateShootingPose),
                    new SetArmPosition(arm, drive::getArmShootingAngle),
                    new IntakeCommand(shooter))
                .withTimeout(2.0),
            new ShootCommand(shooter, arm, drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .onlyWhile(() -> shooter.intakeSensor.get());
  }

  public static Command aimArmAndShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new SetArmPosition(arm, drive::getArmShootingAngle),
            new ShootCommand(shooter, arm, drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .withTimeout(1.5);
  }

  public static Command shuttleShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelCommandGroup(
        new DriveToPosition(drive, drive::calculateShuttlePose),
        new SetArmPosition(arm, () -> ArmConstants.kArmPoseIntake),
        new IntakeCommand(shooter));
  }
}
