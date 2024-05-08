package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.NoteVisualizer;

/**
 * A class of commands that all affect the orientation of the robot, the angle of the arm, and the
 * shooter toggle to automatically fire towards the desired target. Instead of directly aiming at
 * the apriltags, we use the robot's odometry to determine how it should aim.
 */
public class ShooterTargeting {

  /** The manual autoshoot command directly into the speaker. Used in teleo. */
  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new DriveToPosition(drive, drive::calculateShootingPose),
                    new SetArmPosition(arm, drive::getArmShootingAngle),
                    new IntakeCommand(shooter))
                .onlyWhile(() -> shooter.intakeSensor.get())
                .withTimeout(2.0),
            new ShootCommand(shooter, arm, drive),
            NoteVisualizer.shoot(drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .onlyWhile(() -> shooter.intakeSensor.get());
  }

  /** The automatic autoshoot command directly into the speaker. Used in autonomous. */
  public static Command aimArmAndShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new SetArmPosition(arm, drive::getArmShootingAngle),
            new ShootCommand(shooter, arm, drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .withTimeout(1.5);
  }
  /**
   * The manual autoshuttle command directly into the corner of our alliances wing. Used in teleop.
   */
  public static Command shuttleShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelCommandGroup(
        new DriveToPosition(drive, drive::calculateShuttlePose),
        new SetArmPosition(arm, () -> ArmConstants.kArmPoseIntake),
        new IntakeCommand(shooter));
  }
}
