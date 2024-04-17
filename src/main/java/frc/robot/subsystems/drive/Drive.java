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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LocalADStarAK;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  // public static final double MAX_AUTO_SPEED = Units.feetToMeters(7.0);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(24.625);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.625);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  public boolean isUsingVision = false;
  public double armAngleOffset = 3.0;

  public static boolean inShuttlePosition = false;

  private InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  private Field2d m_field = new Field2d();
  private Alliance allianceColor;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, 0.05),
          VecBuilder.fill(0.5, 0.5, 2.0));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    SmartDashboard.putData("Field", m_field);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        this.allianceColor = Alliance.Red;
      }
      if (ally.get() == Alliance.Blue) {
        this.allianceColor = Alliance.Blue;
      }
    }

    for (double[] d : Constants.ArmConstants.shooterValues) {
      shooterTable.put(d[0], d[1]);
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    // double[] sampleTimestamps =
    //    modules[0].getOdometryTimestamps(); // This assumes that all module signals are sampled
    // together?
    //    int sampleCount = sampleTimestamps.length;
    int sampleCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      // Determine the minimum number of odometry samples (across swerve modules and gyro
      // timestamps)
      sampleCount = Math.min(sampleCount, modules[i].getOdometryPositions().length);
    }

    // Average the timestamp values
    double[] mod0timeStamps = modules[0].getOdometryTimestamps();
    double[] mod1timeStamps = modules[1].getOdometryTimestamps();
    double[] mod2timeStamps = modules[2].getOdometryTimestamps();
    double[] mod3timeStamps = modules[3].getOdometryTimestamps();

    double[] sampleTimestamps = new double[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      sampleTimestamps[i] =
          (mod0timeStamps[i] + mod1timeStamps[i] + mod2timeStamps[i] + mod3timeStamps[i]);
      if (gyroInputs.connected) {
        sampleTimestamps[i] += gyroInputs.odometryYawTimestamps[i];
        sampleTimestamps[i] /= 5.0; // Average across 5 values (4 modules and gyro yaw)
      } else {
        sampleTimestamps[i] /= 4.0; // Average across only 4 values (4 modules)
      }
    }
    // Logger.recordOutput("Odometry/Timestamps", sampleTimestamps);

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    m_field.setRobotPose(poseEstimator.getEstimatedPosition());

    // SmartDashboard.putNumber("Gyro Yaw", gyroInputs.yawPosition.getDegrees());
    // SmartDashboard.putNumber("FL encoder val", modules[0].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("FR encoder val", modules[1].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("BL encoder val", modules[2].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("BR encoder val", modules[3].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Odometry X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Odometry Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putBoolean("Vision On?", isUsingVision);
    SmartDashboard.putNumber("Arm angle offset", armAngleOffset);

    SmartDashboard.putBoolean("Gyro connected?", gyroInputs.connected);

    Logger.recordOutput("Drive/Gyro Yaw", gyroInputs.yawPosition.getDegrees());
    // Logger.recordOutput("FL encoder val", modules[0].getPosition().angle.getDegrees());
    // Logger.recordOutput("FR encoder val", modules[1].getPosition().angle.getDegrees());
    // Logger.recordOutput("BL encoder val", modules[2].getPosition().angle.getDegrees());
    // Logger.recordOutput("BR encoder val", modules[3].getPosition().angle.getDegrees());
    // Logger.recordOutput("Odometry X", poseEstimator.getEstimatedPosition().getX());
    // Logger.recordOutput("Odometry Y", poseEstimator.getEstimatedPosition().getY());
    Logger.recordOutput("Vision On?", isUsingVision);
    Logger.recordOutput("Arm/Arm angle offset", armAngleOffset);

    Boolean redVar;
    if (Robot.isReal()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        redVar = true;
        this.allianceColor = Alliance.Red;
      } else {
        redVar = false;
        this.allianceColor = Alliance.Blue;
      }
    } else {
      if (DriverStationSim.getAllianceStationId().compareTo(AllianceStationID.Blue1) < 0) {
        this.allianceColor = Alliance.Red;
        redVar = true;
      } else {
        this.allianceColor = Alliance.Blue;
        redVar = false;
      }
    }

    Logger.recordOutput("Alliance Red?", redVar);
    SmartDashboard.putBoolean("Alliance Red?", redVar);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public double[] getVelocity() {
    double[] vec = {
      kinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond,
      kinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond,
    };
    return vec;
  }

  public double getVelocityX() {
    return getVelocity()[0];
  }

  public double getVelocityY() {
    return getVelocity()[1];
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive velocities) for all of the modules. */
  // @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Translation2d getTranslation() {
    return poseEstimator.getEstimatedPosition().getTranslation();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void zeroGyro() {
    gyroIO.zeroGyro();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * log Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    if (isUsingVision) poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  // The following three functions are all for shooting calculations for both

  // Use lerp table to get arm angle for autoshoot
  public double getArmShootingAngle() {
    Pose2d current = getPose();
    Translation2d difference =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red)
            ? Constants.FieldConstants.kSpeakerTargetPoseRed.minus(current.getTranslation())
            : Constants.FieldConstants.kSpeakerTargetPoseBlue.minus(current.getTranslation());
    double distance = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
    double armDegrees =
        shooterTable.get(distance) + armAngleOffset; // armAngleOffset should start at 0.0
    armDegrees = MathUtil.clamp(armDegrees, 0.0, 90.0);

    Logger.recordOutput("AutoShoot/Distance X", difference.getX());
    Logger.recordOutput("AutoShoot/Distance Y", difference.getY());
    Logger.recordOutput("AutoShoot/Target Arm Angle", armDegrees);

    return armDegrees;
  }

  // used to calculate shooting for teleop autoshoot
  public Pose2d calculateShootingPose() {
    Translation2d current = calculateProjectedRobotPose();
    Translation2d goal =
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red
                    || allianceColor == Alliance.Red))
            ? Constants.FieldConstants.kSpeakerTargetPoseRed
            : Constants.FieldConstants.kSpeakerTargetPoseBlue;
    goal = current.minus(goal);
    double angle = Math.atan(goal.getY() / goal.getX());
    return new Pose2d(getPose().getTranslation(), new Rotation2d(angle));
  }

  // used in teleop to aim at the amp and shoot funneling shots.
  public Pose2d calculateShuttlePose() {
    Translation2d current = calculateProjectedRobotPose();
    Translation2d goal =
        (DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == Alliance.Red
                    || allianceColor == Alliance.Red))
            ? Constants.FieldConstants.kAmpTargetPoseRed
            : Constants.FieldConstants.kAmpTargetPoseBlue;
    goal = current.minus(goal);
    double angle = Math.atan(goal.getY() / goal.getX());
    return new Pose2d(getPose().getTranslation(), new Rotation2d(angle));
  }

  public Pose2d getShuttlePoseConstant() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI / 5));
    } else {
      return new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(-Math.PI / 5));
    }
  }

  public Pose2d calculateIntakePose() {
    Pose2d current = getPose();
    return new Pose2d(
        current.getTranslation(),
        current
            .getRotation()
            .minus(
                new Rotation2d(
                    Math.toRadians(LimelightVisionSubsystem.limelightNoteTrack() / 1.1))));
  }

  public Optional<Rotation2d> calculateIntakeAngle() {
    Pose2d pose = calculateIntakePose();
    Optional<Rotation2d> angle = Optional.of(pose.getRotation());
    // Might have to add the different case for red side, address this after krytpon
    return angle;
  }

  public Pose2d calculateShootingDirectPose() {
    Pose2d current = getPose();

    if (Math.abs(VisionSubsystem.yawOffset) > 10) {
      return new Pose2d(
          current.getTranslation(),
          current
              .getRotation()
              .minus(new Rotation2d(Math.toRadians(VisionSubsystem.yawOffset / 5))));
    } else {
      return new Pose2d(
          current.getTranslation(),
          current
              .getRotation()
              .minus(new Rotation2d(Math.toRadians(VisionSubsystem.yawOffset / 2))));
    }
  }

  // Used in autonomous paths, not in teleop
  public Optional<Rotation2d> calculateShootingAngle() {
    Optional<Rotation2d> angle = Optional.of(calculateShootingPose().getRotation());
    if (DriverStation.getAlliance().isPresent()
        && (DriverStation.getAlliance().get() == Alliance.Red || allianceColor == Alliance.Red)
        && angle.isPresent()) {
      angle = Optional.of(new Rotation2d(Math.PI).plus(angle.get()));
    }
    return (angle.isPresent()) ? angle : Optional.empty();
  }

  // calculates the predicted robot pose by the time the shot meets the target, allowing us to move
  // and shoot.
  public Translation2d calculateProjectedRobotPose() {
    var invert = (allianceColor == Alliance.Red) ? -1.0 : 1.0;
    Translation2d originalTargetTranslation =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red)
            ? Constants.FieldConstants.kSpeakerTargetPoseRed
            : Constants.FieldConstants.kSpeakerTargetPoseBlue;
    Translation2d currentRobotTranslation = getTranslation();
    double Vs = 10.0; // m/s
    double Xr = currentRobotTranslation.getX();
    double Yr = currentRobotTranslation.getY();
    double Xt = originalTargetTranslation.getX();
    double Yt = originalTargetTranslation.getY();
    double Vx = kinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond;
    double Vy = kinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond;
    double k1 = (sqr(Xt) + sqr(Xr) - 2 * Xr * Xt + sqr(Yt) + sqr(Yr) - 2 * Yr * Yt);
    double k2 = (2 * Vx * Xt - 2 * Vx * Xr + 2 * Vy * Yt - 2 * Vy * Yr);
    double k3 = (sqr(Vs) - (sqr(Vx) + sqr(Vy)));
    double Ts = ((k2 + sqrt(sqr(k2) + 4 * k3 * k1)) / (2 * k3));
    Translation2d projTarget =
        new Translation2d(
            sqr(Vx) * Math.signum(Vx) * Ts * invert + Xr,
            sqr(Vy) * Math.signum(Vy) * Ts * invert + Yr);
    SmartDashboard.putNumber("New Target X", projTarget.getY());
    Logger.recordOutput("projTarget", projTarget);
    return projTarget;
  }

  private double sqr(double x) {
    return Math.pow(x, 2);
  }

  private double sqrt(double x) {
    return Math.pow(x, 0.5);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estStdDevs) {
    if (isUsingVision) poseEstimator.addVisionMeasurement(visionPose, timestamp, estStdDevs);
    m_field.getObject("vision estimate").setPose(visionPose);
    SmartDashboard.putNumber("Vision pose X:", visionPose.getX());
    SmartDashboard.putNumber("Vision pose Y:", visionPose.getY());
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public Pose2d[] getModulePoses() {
    Pose2d[] modulePoses = new Pose2d[modules.length];
    for (int i = 0; i < modules.length; i++) {
      var module = modules[i];
      modulePoses[i] =
          getPose().transformBy(new Transform2d(getModuleTranslations()[i], module.getAngle()));
    }
    return modulePoses;
  }
}
