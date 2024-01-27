package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  private final TalonFX m_ArmMotor1 = new TalonFX(15, "rio");

  public WinchSubsystem() {
    TalonUtil.setBrakeMode(m_ArmMotor1);
  }

  public void setWinchSpeed(double speed) {
    m_ArmMotor1.set(speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Value", m_ArmMotor1.getPosition().getValueAsDouble());
  }
}
