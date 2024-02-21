package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  public final DigitalInput motor12LowerLimitSwitch = new DigitalInput(3);
  public final DigitalInput motor12UpperLimitSwitch = new DigitalInput(5);
  public final DigitalInput motor11LowerLimitSwitch = new DigitalInput(4);
  public final DigitalInput motor11UpperLimitSwitch = new DigitalInput(6);

  public final TalonFX m_WinchMotorRight = new TalonFX(12, "rio");
  public final TalonFX m_WinchMotorLeft = new TalonFX(11, "rio");

  public final Servo m_WinchRightServo = new Servo(0);
  public final Servo m_WinchLeftServo = new Servo(1);

  private VelocityVoltage m_request = new VelocityVoltage(0);

  public boolean winchEnabled = true;

  public ClimbSubsystem() {
    m_WinchMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_WinchMotorRight.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setRightWinchSpeed(double speed) {
    if (!winchEnabled) {
      m_WinchMotorRight.set(0);
      m_WinchMotorLeft.set(0);
      return;
    }

    if ((speed < 0.0 && !motor12LowerLimitSwitch.get())
        || (speed > 0.0 && !motor12UpperLimitSwitch.get())) {
      speed = 0;
    }

    m_WinchMotorRight.set(speed);
  }

  public void setLeftWinchSpeed(double speed) {

    if (!winchEnabled) {
      m_WinchMotorRight.set(0);
      m_WinchMotorLeft.set(0);
      return;
    }

    if ((speed < 0.0 && !motor11LowerLimitSwitch.get())
        || (speed > 0.0 && !motor11UpperLimitSwitch.get())) {
      speed = 0;
    }

    m_WinchMotorLeft.set(-speed);
  }

  public void setWinchVelocity(double velocity) {

    double leftSpeed = velocity;
    double rightSpeed = velocity;

    if (!winchEnabled) {
      m_WinchMotorRight.setControl(m_request.withVelocity(0));
      m_WinchMotorLeft.setControl(m_request.withVelocity(0));
      return;
    }

    if ((leftSpeed < 0.0 && !motor11LowerLimitSwitch.get())
        || (leftSpeed > 0.0 && !motor11UpperLimitSwitch.get())) {
      leftSpeed = 0;
    }

    if ((rightSpeed < 0.0 && !motor12LowerLimitSwitch.get())
        || (rightSpeed > 0.0 && !motor12UpperLimitSwitch.get())) {
      rightSpeed = 0;
    }

    m_WinchMotorRight.setControl(m_request.withVelocity(rightSpeed));
    m_WinchMotorLeft.setControl(m_request.withVelocity(-leftSpeed));
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
    SmartDashboard.putBoolean("4", motor11UpperLimitSwitch.get());
    SmartDashboard.putBoolean("6", motor11LowerLimitSwitch.get());
    SmartDashboard.putBoolean("3", motor12LowerLimitSwitch.get());
    SmartDashboard.putBoolean("5", motor12UpperLimitSwitch.get());
  }
}
