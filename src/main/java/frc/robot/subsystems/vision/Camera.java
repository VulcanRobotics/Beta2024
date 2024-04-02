package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
  private CameraIO cameraIO;
  private final CameraIOInputs cameraInputs = new CameraIOInputs();
  private final int cameraIndex;

  public Camera(Drive robotDrive, int index) {
    cameraIndex = index;
    if (Constants.currentMode != Constants.Mode.SIM) {
      cameraIO = new CameraIOPhoton(robotDrive, cameraIndex);
    }
  }

  public Camera(Drive robotDrive, int index, VisionSystemSim visionSim) {
    cameraIndex = index;
    if (Constants.currentMode == Constants.Mode.SIM) {
      cameraIO = new CameraIOPhotonSim(robotDrive, index, visionSim);
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return cameraIO.getLatestResult();
  }

  public void periodic() {
    cameraIO.updateInputs(cameraInputs);
    Logger.processInputs("Vision/Camera" + Integer.toString(cameraIndex), cameraInputs);
  }
}
