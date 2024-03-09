/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
// import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionSubsystem extends SubsystemBase {
  // private final PhotonCamera camera;
  private final PhotonCamera cameraFL;
  private final PhotonCamera cameraFR;
  private final PhotonCamera cameraBL;
  private final PhotonCamera cameraBR;

  private enum CameraPosition {
    FL,
    FR,
    BL,
    BR
  };

  // private final PhotonPoseEstimator photonEstimator;
  private final PhotonPoseEstimator photonEstimatorFL;
  private final PhotonPoseEstimator photonEstimatorFR;
  private final PhotonPoseEstimator photonEstimatorBL;
  private final PhotonPoseEstimator photonEstimatorBR;

  // Last vision update timestamps, per camera
  private double lastEstTimestamp = 0;
  private double lastEstTimestampFL = 0;
  private double lastEstTimestampFR = 0;
  private double lastEstTimestampBL = 0;
  private double lastEstTimestampBR = 0;

  private Drive drive;

  // Simulation
  // private PhotonCameraSim cameraSim;
  private PhotonCameraSim cameraSimFL;
  private PhotonCameraSim cameraSimFR;
  private PhotonCameraSim cameraSimBL;
  private PhotonCameraSim cameraSimBR;
  private VisionSystemSim visionSim;

  // The layout of the AprilTags on the field
  public AprilTagFieldLayout kTagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

  public PhotonVisionSubsystem(Drive robotDrive) {
    drive = robotDrive;
    /*
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        kTagLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
      if (ally.get() == Alliance.Blue) {
        kTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
    }
    */

    // Make sure that this is actually aligned with the correct alliance
    // camera = new PhotonCamera(kCameraName);
    cameraFL = new PhotonCamera(kCameraNameFL);
    cameraFR = new PhotonCamera(kCameraNameFR);
    cameraBL = new PhotonCamera(kCameraNameBL);
    cameraBR = new PhotonCamera(kCameraNameBR);

    /* photonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); */

    photonEstimatorFL =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFL, kRobotToCamFL);
    photonEstimatorFL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorFR =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFR, kRobotToCamFR);
    photonEstimatorFR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorBL =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraBL, kRobotToCamBL);
    photonEstimatorBL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorBR =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraBR, kRobotToCamBR);
    photonEstimatorBR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // ----- Simulation
    if (Robot.isSimulation()) {
      // if (Constants.currentMode == Constants.Mode.SIM) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);
      // Create simulated camera properties. These can be set to mimic your actual camera.
      var cameraProp = new SimCameraProperties();
      // Limelight has horizontal FOV of 63.3 degrees
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(15);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
      // targets.
      // cameraSim = new PhotonCameraSim(camera, cameraProp);

      cameraSimFL = new PhotonCameraSim(cameraFL, cameraProp);
      cameraSimFR = new PhotonCameraSim(cameraFR, cameraProp);
      // cameraSimBL = new PhotonCameraSim(cameraBL, cameraProp);
      // cameraSimBR = new PhotonCameraSim(cameraBR, cameraProp);

      // Add the simulated camera to view the targets on this simulated field.
      // visionSim.addCamera(cameraSim, kRobotToCam);

      visionSim.addCamera(cameraSimFL, kRobotToCamFL);
      visionSim.addCamera(cameraSimFR, kRobotToCamFR);
      // visionSim.addCamera(cameraSimBL, kRobotToCamBL);
      // visionSim.addCamera(cameraSimBR, kRobotToCamBR);

      // cameraSim.enableDrawWireframe(true);

      cameraSimFL.enableDrawWireframe(true);
      cameraSimFR.enableDrawWireframe(true);
      // cameraSimBL.enableDrawWireframe(true);
      // cameraSimBR.enableDrawWireframe(true);
    }
  }

  /* public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  } */

  public PhotonPipelineResult getLatestResult(CameraPosition camPos) {
    PhotonCamera thisCamera;
    switch (camPos) {
      default: // Front-Left
        thisCamera = cameraFL;
        break;
      case FR:
        thisCamera = cameraFR;
        break;
      case BL:
        thisCamera = cameraBL;
        break;
      case BR:
        thisCamera = cameraBR;
        break;
        /* default:
        thisCamera = camera;
        break; */
    }
    return thisCamera.getLatestResult();
  }
  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */

  /* public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    // System.out.print("GetEstimatedGlobalPose()");
    // SmartDashboard.putBoolean("visionEst present:", visionEst.isPresent());

    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
      est ->
          getSimDebugField()
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
      () -> {
        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
      });
    }
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  } */

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(CameraPosition camPos) {
    Optional<EstimatedRobotPose> visionEst;
    double latestTimestamp;
    boolean newResult;
    String fieldObjectName;

    switch (camPos) {
      default: // Front-left
        visionEst = photonEstimatorFL.update();
        SmartDashboard.putBoolean("visionEst FL:", visionEst.isPresent());
        Logger.recordOutput("visionEst FL", visionEst.isPresent());
        latestTimestamp = cameraFL.getLatestResult().getTimestampSeconds();
        newResult = Math.abs(latestTimestamp - lastEstTimestampFL) > 1e-5;
        fieldObjectName = "VisionEstFL";
        break;
      case FR:
        visionEst = photonEstimatorFR.update();
        SmartDashboard.putBoolean("visionEst FR:", visionEst.isPresent());
        Logger.recordOutput("visionEst FR", visionEst.isPresent());
        latestTimestamp = cameraFR.getLatestResult().getTimestampSeconds();
        newResult = Math.abs(latestTimestamp - lastEstTimestampFR) > 1e-5;
        fieldObjectName = "VisionEstFR";
        break;
      case BL:
        visionEst = photonEstimatorBL.update();
        Logger.recordOutput("visionEst BL", visionEst.isPresent());
        latestTimestamp = cameraBL.getLatestResult().getTimestampSeconds();
        newResult = Math.abs(latestTimestamp - lastEstTimestampBL) > 1e-5;
        fieldObjectName = "VisionEstBL";
        break;
      case BR:
        visionEst = photonEstimatorBR.update();
        Logger.recordOutput("visionEst BR", visionEst.isPresent());
        latestTimestamp = cameraBR.getLatestResult().getTimestampSeconds();
        newResult = Math.abs(latestTimestamp - lastEstTimestampBR) > 1e-5;
        fieldObjectName = "VisionEstBR";
        break;
        /* default:
        visionEst = photonEstimator.update();
        latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        fieldObjectName = "VisionEstimation";
        break; */
    }

    if (Robot.isSimulation()) {
      /* visionEst.ifPresentOrElse(
      est ->
          getSimDebugField().getObject(fieldObjectName).setPose(est.estimatedPose.toPose2d()),
      () -> {
        if (newResult) {
          getSimDebugField().getObject(fieldObjectName).setPoses();
        }
      }); */
    }

    if (newResult) {
      switch (camPos) {
        case FL:
          lastEstTimestampFL = latestTimestamp;
          break;
        case FR:
          lastEstTimestampFR = latestTimestamp;
          break;
        case BL:
          lastEstTimestampBL = latestTimestamp;
          break;
        case BR:
          lastEstTimestampBR = latestTimestamp;
          break;
      }
    }

    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */

  /*
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  } */

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, CameraPosition camPos) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = getLatestResult(camPos).getTargets();
    int numTags = 0;
    PhotonPoseEstimator thisPoseEstimator;

    switch (camPos) {
      default: // Front-left
        thisPoseEstimator = photonEstimatorFL;
        break;
      case FR:
        thisPoseEstimator = photonEstimatorFR;
        break;
      case BL:
        thisPoseEstimator = photonEstimatorBL;
        break;
      case BR:
        thisPoseEstimator = photonEstimatorBR;
        break;
        /* default:
        thisPoseEstimator = photonEstimator;
        break; */
    }

    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = thisPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  // ----- Simulation

  @Override
  public void periodic() {

    // Correct pose estimate with vision measurements
    // var visionEst = getEstimatedGlobalPose();
    var visionEstFL = getEstimatedGlobalPose(CameraPosition.FL);
    var visionEstFR = getEstimatedGlobalPose(CameraPosition.FR);
    var visionEstBL = getEstimatedGlobalPose(CameraPosition.BL);
    var visionEstBR = getEstimatedGlobalPose(CameraPosition.BR);

    /* visionEst.ifPresent(
    est -> {
      // System.out.print("PhotonVisionSubsystem ifPresent()");
      var estPose = est.estimatedPose.toPose2d();
      // Change our trust in the measurement based on the tags we can see
      var estStdDevs = getEstimationStdDevs(estPose);

      // Updated to take advantage of new AdvancedSwerveDrive Project, which incorporates
      // a SwerveDrivePoseEstimator in Drive, making it easy to update the pose position
      // with vision estimates.
      drive.addVisionMeasurement(
          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

      // var debugField = getSimDebugField();
      // debugField.getObject("EstimatedRobot").setPose(estPose);

      // SmartDashboard.putNumber("Vision pose X:", estPose.getX());
      // SmartDashboard.putNumber("Vision pose Y:", estPose.getY());
      // debugField.getObject("EstimatedRobotModules").setPoses(drive.getModulePoses());
    }); */

    /* visionEst.ifPresent(
    est -> {
      var estPose = est.estimatedPose.toPose2d();
      Logger.recordOutput("Vision/estPoseBack", estPose);
      var estStdDevs = getEstimationStdDevs(estPose);
      drive.addVisionMeasurement(
          est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    }); */

    visionEstFL.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          Logger.recordOutput("Vision/estPoseFL", estPose);
          var estStdDevs = getEstimationStdDevs(estPose, CameraPosition.FL);
          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    visionEstFR.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          Logger.recordOutput("Vision/estPoseFR", estPose);
          var estStdDevs = getEstimationStdDevs(estPose, CameraPosition.FR);
          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    visionEstBL.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          Logger.recordOutput("Vision/estPoseBL", estPose);
          var estStdDevs = getEstimationStdDevs(estPose, CameraPosition.BL);
          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });

    visionEstBR.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          Logger.recordOutput("Vision/estPoseBR", estPose);
          var estStdDevs = getEstimationStdDevs(estPose, CameraPosition.BR);
          drive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
  }

  @Override
  //  public void simulationPeriodic(Pose2d robotSimPose) {
  public void simulationPeriodic() {
    // visionSim.update(robotSimPose);
    visionSim.update(drive.getPose());

    var debugField = getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(drive.getPose());
    debugField.getObject("EstimatedRobotModules").setPoses(drive.getModulePoses());
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    // if (Robot.isSimulation())
    if (Constants.currentMode == Constants.Mode.SIM) {
      visionSim.resetRobotPose(pose);
    }
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
