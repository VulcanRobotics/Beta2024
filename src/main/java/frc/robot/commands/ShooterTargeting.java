package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

/** This class contains command compositions for different shooting functions */
public class ShooterTargeting {

  /** Aims at the speaker and shoots the note, stops and continues if there is no note present */
  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new DriveToPosition(drive, drive::calculateShootingPose),
                    new SetArmPosition(arm, drive::getArmShootingAngle),
                    new IntakeCommand(shooter))
                .onlyWhile(() -> shooter.intakeSensor.get())
                .withTimeout(2.0),
            new ShootCommand(shooter, arm, drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .onlyWhile(() -> shooter.intakeSensor.get());
  }
  /** Only move the arm and shoots. Use this if the rotation is controlled independently */
  public static Command aimArmAndShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new SequentialCommandGroup(
            new SetArmPosition(arm, drive::getArmShootingAngle),
            new ShootCommand(shooter, arm, drive))
        .onlyWhile(() -> shooter.intakeSensor.get())
        .withTimeout(1.5);
  }
  /** Used to pass notes from the center line to the amp/speaker area on our side of the field. */
  public static Command shuttleShoot(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelCommandGroup(
        new DriveToPosition(drive, drive::calculateShuttlePose),
        new SetArmPosition(arm, () -> ArmConstants.kArmPoseIntake),
        new IntakeCommand(shooter));
  }
}
