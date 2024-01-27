package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.*;

public class WinchSubsystem extends SubsystemBase {
  public final TalonFX m_ArmMotor1 = new TalonFX(15, "rio");
  private double encoderOffset = 0;

  public WinchSubsystem() {
    TalonUtil.setBrakeMode(m_ArmMotor1);
  }

  public double getArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble() - encoderOffset;
  }

  public void updateEncoderOffset() {
    encoderOffset = m_ArmMotor1.getPosition().getValueAsDouble();
  }

  public void setWinchSpeed(double speed) {
    m_ArmMotor1.set(0.2 * speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Value", getArmEncoder());
    SmartDashboard.putString("Brake Mode", m_ArmMotor1.getConfigurator().toString());
  }
}
