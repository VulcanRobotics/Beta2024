package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LimelightVisionSubsystem extends SubsystemBase {

  public LimelightVisionSubsystem() {
    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
    }
  }

  public static double limelightNoteTrack() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    // read values periodically
    double x = tx.getDouble(0.0);

    // post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);

    return x;
  }
}
