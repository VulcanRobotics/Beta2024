package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/** Command for driving to a specified position on the field and stopping. Code inspired by 2539. */
public class DriveToPosition extends Command {

  private final double AnglePIDValues[] = {0.5, 0.0, 0.0};
  private final double DrivePIDValues[] = {1.2, 0.0, 0.1};

  private PIDController xController =
      new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
  private PIDController yController =
      new PIDController(DrivePIDValues[0], DrivePIDValues[1], DrivePIDValues[2]);
  private PIDController mAngleController =
      new PIDController(AnglePIDValues[0], AnglePIDValues[1], AnglePIDValues[2]);
  /// new Constraints(Drive.MAX_ANGULAR_SPEED, Drive.MAX_ANGULAR_SPEED)); // Fix this

  private final Drive swerveDriveSubsystem;
  private Supplier<Pose2d> targetPoseSupplier;

  public DriveToPosition(Drive swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier) {
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
    mAngleController.reset();

    xController.setTolerance(Units.inchesToMeters(0.5));
    yController.setTolerance(Units.inchesToMeters(0.5));

    atX = atY = atTheta = false;
  }

  @Override
  public void execute() {
    Pose2d robotPose = swerveDriveSubsystem.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    mAngleController.setSetpoint(
        /*(DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red)
        ? targetPose.getRotation().getRadians() + Math.PI
        : */ targetPose.getRotation().getRadians());

    // Drive to the target
    var xSpeed = xController.calculate(robotPose.getX());
    var ySpeed = yController.calculate(robotPose.getY());

    var thetaSpeed =
        mAngleController.calculate(
            robotPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
                + (DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red
                    ? Math.PI
                    : 0.0));

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

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    swerveDriveSubsystem.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            ySpeed * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            thetaSpeed * swerveDriveSubsystem.getMaxAngularSpeedRadPerSec(),
            (isFlipped)
                ? swerveDriveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
                : swerveDriveSubsystem.getRotation()));
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
