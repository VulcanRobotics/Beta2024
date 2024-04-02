package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  private final Camera[] cameras = new Camera[4];

  /* private enum CameraPosition {
    FL,
    FR,
    BL,
    BR
  }; */

  private Drive drive;
  private VisionSystemSim visionSim;
  public static double yawOffset;

  // The layout of the AprilTags on the field
  public AprilTagFieldLayout kTagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());

  public VisionSubsystem(Drive robotDrive) {
    drive = robotDrive;

    // ----- Simulation
    // if (Robot.isSimulation()) {
    if (Constants.currentMode == Constants.Mode.SIM) { // SIM only, not REPLAY
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);

      for (int n = 0; n < 4; n++) {
        cameras[n] = new Camera(drive, n, visionSim);
      }
    } else {
      for (int n = 0; n < 4; n++) {
        cameras[n] = new Camera(drive, n);
      }
    }
  }

  @Override
  public void periodic() {
    for (int n = 0; n < 4; n++) {
      cameras[n].periodic();
    }

    PhotonPipelineResult latestResult = cameras[3].getLatestResult();

    if (latestResult.hasTargets()) {
      if (latestResult.getBestTarget().getFiducialId() == 7) {
        yawOffset = latestResult.getBestTarget().getYaw();
        // SmartDashboard.putNumber("YawOffset", yawOffset);
      }
    }
  }

  @Override
  //  public void simulationPeriodic(Pose2d robotSimPose) {
  public void simulationPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) { // Not REPLAY
      visionSim.update(drive.getPose());

      var debugField = getSimDebugField();
      debugField.getObject("EstimatedRobot").setPose(drive.getPose());
      debugField.getObject("EstimatedRobotModules").setPoses(drive.getModulePoses());
    }
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
