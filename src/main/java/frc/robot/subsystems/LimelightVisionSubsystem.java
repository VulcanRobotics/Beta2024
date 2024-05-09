package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Optional;

public class LimelightVisionSubsystem extends SubsystemBase {

  public LimelightVisionSubsystem() {
    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
    }
  }
  /** Grabs the x value from the limelight and returns it for other functions to uitilize */
  public static double limelightNoteTrack() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  /**
   * Grabs the x value from the limelight and returns it for other functions to uitilize. It does
   * return empty though if the camera does not see any notes (returning an x value of 0).
   */
  public static Optional<Double> getNoteDistLL() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    return (x == 0.0) ? Optional.empty() : Optional.of(x);
  }
}
