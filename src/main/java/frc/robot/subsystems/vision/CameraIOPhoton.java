package frc.robot.subsystems.vision;


import static frc.robot.Constants.Vision.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public class CameraIOPhoton implements CameraIO {
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;
    protected Transform3d robotToCam;

    // The layout of the AprilTags on the field
    public AprilTagFieldLayout kTagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

    public CameraIOPhoton(int index) {
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
