package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOPhotonSim extends CameraIOPhoton {
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  public CameraIOPhotonSim(Drive robotDrive, int index, VisionSystemSim visSim) {
    super(robotDrive, index);

    visionSim = visSim;

    // Create simulated camera properties.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 400, Rotation2d.fromDegrees(67));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(120);
    cameraProp.setAvgLatencyMs(15);
    cameraProp.setLatencyStdDevMs(5);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, robotToCam);
    cameraSim.enableDrawWireframe(true);
  }

  @Override
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult pipelineResult) {
    Optional<EstimatedRobotPose> visionEst = photonEstimator.update(pipelineResult);

    Field2d simField = visionSim.getDebugField();
    Boolean newResult = (Math.abs(pipelineResult.getTimestampSeconds() - lastEstTimestamp) > 1e-5);

    visionEst.ifPresentOrElse(
        est -> simField.getObject(cameraName).setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult) {
            simField.getObject(cameraName).setPoses();
          }
        });

    return visionEst;
  }
}
