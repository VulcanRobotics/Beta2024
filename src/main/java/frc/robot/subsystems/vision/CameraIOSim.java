package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;
import frc.robot.Robot;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private final PhotonCameraSim cameraSim;
    
    public CameraIOPhotonSim(Drive robotDrive, int index) {
        super(drive, index);

        switch (index) {
            default:
                camera = new PhotonCamera(kCameraNameFL);
                robotToCam = kRobotToCamFL;
                break;
            case 1:
                camera = new PhotonCamera(kCameraNameFR);
                robotToCam = kRobotToCamFR;
                break;
            case 2:
                camera = new PhotonCamera(kCameraNameBL);
                robotToCam = kRobotToCamBL;
                break;
            case 3:
                camera = new PhotonCamera(kCameraNameBR);
                robotToCam = kRobotToCamBR;
                break;
        }

        photonEstimator = new PhotonPoseEstimator(
            kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create simulated camera properties. These can be set to mimic your actual camera.
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
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst;
        double latestTimestamp;

        visionEst = photonEstimator.update();
        if ( visionEst.isPresent() ) {
            latestTimestamp = camera.getLatestResult().getTimestampSeconds();
            if ( Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5 ) {
                lastEstTimestamp = latestTimestamp;
            }
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
      
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
    
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
    
        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;
        
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 45));

        return estStdDevs;
    }
}
