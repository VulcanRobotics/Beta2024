package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.*;

public class ClimbSubsystem extends SubsystemBase {
  public final TalonFX m_WinchMotor = new TalonFX(12, "rio");

  public void setWinchSpeed(double speed) {
    m_WinchMotor.set(speed);
  }
}
