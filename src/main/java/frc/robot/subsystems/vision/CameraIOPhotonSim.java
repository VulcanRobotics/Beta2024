package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    
    public CameraIOPhotonSim(int index, VisionSystemSim visSim) {
        super(index);

        visionSim = visSim;

        // Create simulated camera properties.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 400, Rotation2d.fromDegrees(67));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(50);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(5);
        
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, robotToCam);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = super.getEstimatedGlobalPose();
        
        /* visionEst.ifPresentOrElse(
            est ->
                getSimDebugField().getObject(fieldObjectName).setPose(est.estimatedPose.toPose2d()),
            () -> {
              if (newResult) {
                getSimDebugField().getObject(fieldObjectName).setPoses();
              }
            });
        } */
      
        return visionEst;
    }

}
