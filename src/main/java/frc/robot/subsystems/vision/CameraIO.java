package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;


public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean connected = false;
        // public Pose2d estimatedPose = new Pose2d();
        public boolean poseDetected = false;
        public List<PhotonTrackedTarget> targets;
        public double latestTimestamp = 0;
    }

  public default void updateInputs(CameraIOInputs inputs) {}
}
