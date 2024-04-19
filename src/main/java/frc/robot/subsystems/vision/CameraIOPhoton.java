package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhoton implements CameraIO {
  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator photonEstimator;
  protected double lastEstTimestamp = 0;
  protected Transform3d robotToCam;
  private final RawSubscriber rawBytesSubscriber;
  protected String cameraName;
  // private Boolean poseDetected;
  private Drive drive;

  // The layout of the AprilTags on the field
  public AprilTagFieldLayout kTagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

  public CameraIOPhoton(Drive robotDrive, int index) {
    drive = robotDrive;

    switch (index) {
      default:
        cameraName = kCameraNameFL;
        robotToCam = kRobotToCamFL;
        break;
      case 1:
        cameraName = kCameraNameFR;
        robotToCam = kRobotToCamFR;
        break;
      case 2:
        cameraName = kCameraNameBL;
        robotToCam = kRobotToCamBL;
        break;
      case 3:
        cameraName = kCameraNameBR;
        robotToCam = kRobotToCamBR;
        break;
    }

    camera = new PhotonCamera(cameraName);

    photonEstimator =
        new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    rawBytesSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("photonvision")
            .getSubTable(cameraName)
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult pipelineResult) {
    // Optional<EstimatedRobotPose> visionEst;

    return photonEstimator.update(pipelineResult);
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult cameraResult) {
    var estStdDevs = kSingleTagStdDevs;
    // var targets = camera.getLatestResult().getTargets();
    var targets = cameraResult.getTargets();
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
    Logger.recordOutput("Vision/Avg. distance", avgDist);

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 32));

    if (DriverStation.isAutonomous()) {
      estStdDevs = estStdDevs.times(Vision.kVisionAutoMultiplier);
    }

    return estStdDevs;
  }

  @Override
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    if (Constants.currentMode != Constants.Mode.REPLAY) {
      // byte[][] rawBytesFrames = rawBytesSubscriber.readQueueValues();

      // // if (rawBytesFrames.length > 0) {
      // if (inputs.numFrames > 0) {
      //   inputs.rawBytes = rawBytesFrames;
      // }

      TimestampedRaw[] rawBytesFrames = rawBytesSubscriber.readQueue();
      inputs.numFrames = rawBytesFrames.length;
      inputs.rawBytes = new byte[inputs.numFrames][];
      inputs.timestamps = new double[inputs.numFrames];

      for (int n = 0; n < rawBytesFrames.length; n++) {
        inputs.rawBytes[n] = rawBytesFrames[n].value;
        inputs.timestamps[n] = rawBytesFrames[n].timestamp / 1e6;
      }
    }

    if (inputs.numFrames > 0) {
      // if (inputs.rawBytes.length > 0) {
      // byte[] latestFrame = rawBytesFrames[rawBytesFrames.length - 1];

      // byte[] latestFrame = inputs.rawBytes[0];
      byte[] latestFrame = inputs.rawBytes[inputs.numFrames - 1];

      var pipelineResult = PhotonPipelineResult.serde.unpack(new Packet(latestFrame));

      // if (Constants.currentMode != Constants.Mode.REPLAY) {
      // Set the timestamp of the result.
      // getLatestChange returns in microseconds, so we divide by 1e6 to convert to seconds.
      // pipelineResult.setTimestampSeconds(
      //    (rawBytesSubscriber.getLastChange() / 1e6) - pipelineResult.getLatencyMillis() / 1e3);
      pipelineResult.setTimestampSeconds(
          inputs.timestamps[inputs.numFrames - 1] - pipelineResult.getLatencyMillis() / 1e3);
      // inputs.timestamps[0] - pipelineResult.getLatencyMillis() / 1e3);

      // } else {
      //   pipelineResult.setTimestampSeconds(inputs.latestTimestamp);
      // }
      // Optional<EstimatedRobotPose> visionEst = photonEstimator.update(pipelineResult);
      var visionEst = getEstimatedGlobalPose(pipelineResult);

      if (Constants.currentMode != Constants.Mode.REPLAY) {
        inputs.latestTimestamp = pipelineResult.getTimestampSeconds();
        inputs.photonLatency = pipelineResult.getLatencyMillis() / 1e3;
        inputs.poseDetected = false; // Could be changed below
      }

      if (visionEst.isPresent()) {
        var photonPoseEst = visionEst.get();
        var estPose2d = photonPoseEst.estimatedPose.toPose2d();
        var latestTimestamp = pipelineResult.getTimestampSeconds();

        if (Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5) {
          lastEstTimestamp = latestTimestamp;
          var estStdDevs = this.getEstimationStdDevs(estPose2d, pipelineResult);
          drive.addVisionMeasurement(estPose2d, photonPoseEst.timestampSeconds, estStdDevs);

          if (Constants.currentMode != Constants.Mode.REPLAY) {
            inputs.poseDetected = true;
          }

          Logger.recordOutput("Vision/" + cameraName, estPose2d);
          // Logger.recordOutput("Vision/" + cameraName + "/rawBytes", latestFrame);
        }
      }
    }
  }
}
