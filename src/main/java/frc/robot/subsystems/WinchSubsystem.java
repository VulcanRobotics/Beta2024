package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  private final TalonFX m_ArmMotor1 = new TalonFX(15, "rio");

  public WinchSubsystem() {
    var config = new MotorOutputConfigs();
    config.NeutralMode = NeutralModeValue.Brake;
    m_ArmMotor1.getConfigurator().apply(config);
  }

  public void setWinchSpeed(double speed) {
    m_ArmMotor1.set(speed);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Value", m_ArmMotor1.getPosition().getValueAsDouble());
  }
}
