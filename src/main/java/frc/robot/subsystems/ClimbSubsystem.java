package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  public final TalonFX m_WinchMotorRight = new TalonFX(12, "rio");
  public final TalonFX m_WinchMotorLeft = new TalonFX(11, "rio");

  public final Servo m_WinchRightServo = new Servo(0);
  public final Servo m_WinchLeftServo = new Servo(1);

  public boolean winchEnabled = true;

  public ClimbSubsystem() {
    m_WinchMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_WinchMotorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setWinchSpeed(double speed) {
    if (!winchEnabled) {
      m_WinchMotorRight.set(0);
      m_WinchMotorLeft.set(0);

      return;
    }

    m_WinchMotorRight.set(speed);
    m_WinchMotorLeft.set(-speed);
  }

  public void setServoLock(boolean toggle) {
    if (toggle) {
      m_WinchLeftServo.setAngle(60);
      m_WinchRightServo.setAngle(0);
    } else {
      m_WinchLeftServo.setAngle(0);
      m_WinchRightServo.setAngle(120);
    }
  }

  public void periodic() {
    SmartDashboard.putBoolean("Winch Enabled", winchEnabled);
  }
}
