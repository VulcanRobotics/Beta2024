package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

// DO NOT USE THIS COMMAND RIGHT NOW!!!DO NOT USE!!

public class DriveToPosition extends Command {

  private final double AnglePIDValues[] = {4.0, 0.0, 0.2};
  private final double DrivePIDValues[] = {2.0, 0.0, 0.1};

  private PIDController xController =
      new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
  private PIDController yController =
      new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
  private ProfiledPIDController mAngleController =
      new ProfiledPIDController(
          AnglePIDValues[0],
          AnglePIDValues[1],
          AnglePIDValues[2],
          new Constraints(Drive.MAX_LINEAR_SPEED, Drive.MAX_ANGULAR_SPEED)); // Fix this

  private final Drive swerveDriveSubsystem;
  private Pose2d targetPoseSupplier;

  public DriveToPosition(Drive swerveDriveSubsystem, Pose2d targetPoseSupplier) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.targetPoseSupplier = targetPoseSupplier;
    mAngleController.setTolerance(Math.toRadians(0.25));
    mAngleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveDriveSubsystem);
  }

  private boolean atX, atY, atTheta;

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();

    xController.setTolerance(Units.inchesToMeters(2.0));
    yController.setTolerance(Units.inchesToMeters(0.5));

    atX = atY = atTheta = false;
  }

  @Override
  public void execute() {
    Pose2d robotPose = swerveDriveSubsystem.getPose();
    Pose2d targetPose = targetPoseSupplier;

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    mAngleController.setGoal(targetPose.getRotation().getRadians());

    // Drive to the target
    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());

    var thetaSpeed =
        mAngleController.calculate(
            robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (xController.atSetpoint() || atX) {
      xSpeed = 0;
      // atX = true;

    }

    if (yController.atSetpoint() || atY) {
      ySpeed = 0;
    }

    if (mAngleController.atSetpoint() || atTheta) {
      thetaSpeed = 0;
    }

    swerveDriveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            ySpeed * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            thetaSpeed * swerveDriveSubsystem.getMaxAngularSpeedRadPerSec(),
            swerveDriveSubsystem.getRotation()));
  }

  @Override
  public boolean isFinished() {

    return xController.atSetpoint() && yController.atSetpoint() && mAngleController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, swerveDriveSubsystem.getRotation()));
  }
}
