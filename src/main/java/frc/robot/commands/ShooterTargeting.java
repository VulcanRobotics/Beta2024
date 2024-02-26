package frc.robot.commands;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

// plan to place code for autoshoot here; break it up into multiple commands (e.g. arm, shooter,
// turning)
public class ShooterTargeting {

  public static Command shootAtTarget(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return new ParallelCommandGroup(
        new DriveToPosition(drive, () -> calculateShootingPose(drive::getPose)),
        new SetArmPosition(arm, getArmShootingAngle(drive::getPose)),
        new RevCommand(shooter, false),
        new ShootCommand(shooter));
  }

  public static double getArmShootingAngle(Supplier<Pose2d> poseSupplier) {
    Pose2d current = poseSupplier.get();
    Translation2d difference = Constants.FieldConstants.kSpeakerTargetPose.minus(current.getTranslation());
    double distance = Math.sqrt(
      Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
    double armDegrees = 9.04 + 10.6 * Math.log(distance);
    armDegrees = MathUtil.clamp(armDegrees, 0.0, 90.0);
    return armDegrees;
  }

  public static Pose2d calculateShootingPose(Supplier<Pose2d> poseSupplier) {
    Pose2d current = poseSupplier.get();
    Translation2d goal = Constants.FieldConstants.kSpeakerTargetPose; // Speaker position
    Translation2d currentTranslation = current.getTranslation();
    goal = goal.minus(currentTranslation);
    double angle = Math.atan(goal.getY() / goal.getX());
    return new Pose2d(currentTranslation, new Rotation2d(angle));
  }

}
