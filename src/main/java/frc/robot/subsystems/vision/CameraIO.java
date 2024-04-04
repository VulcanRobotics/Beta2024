package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraIO {
  public static class CameraIOInputs implements LoggableInputs {
    // public boolean connected = false;
    // public Pose2d estimatedPose = new Pose2d();
    public boolean poseDetected = false;
    public double latestTimestamp = 0;
    public double photonLatency = 0;
    public double[] timestamps = new double[] {};
    public int numFrames = 0;
    public byte[][] rawBytes = new byte[][] {};

    // These types don't work with AdvantageKit logging
    // public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    // public Optional<EstimatedRobotPose> robotPose = Optional.empty();
    // public EstimatedRobotPose robotPose = new EstimatedRobotPose(null, latestTimestamp, null,
    // null);

    @Override
    public void toLog(LogTable table) {
      table.put("LatestTimestamps", latestTimestamp);
      table.put("Timestamps", timestamps);
      table.put("NumFrames", numFrames);
      table.put("PoseDetected", poseDetected);
      table.put("Latency", photonLatency);
      table.put("FrameCount", rawBytes.length);
      for (int i = 0; i < rawBytes.length; i++) {
        table.put("rawBytes/" + Integer.toString(i), rawBytes[i]);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      latestTimestamp = table.get("LatestTimestamp", latestTimestamp);
      timestamps = table.get("Timestamps", new double[] {});
      numFrames = (int) table.get("FrameCount", 0);
      poseDetected = table.get("PoseDetected", false);
      photonLatency = table.get("Latency", 0.0);
      rawBytes = new byte[numFrames][];
      for (int i = 0; i < numFrames; i++) {
        rawBytes[i] = table.get("rawBytes/" + Integer.toString(i), new byte[] {});
      }
    }
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public abstract PhotonPipelineResult getLatestResult();

  public abstract Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonPipelineResult pipelineResult);
}
