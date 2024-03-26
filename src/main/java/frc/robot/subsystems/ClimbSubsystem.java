package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  // public final DigitalInput motor12LowerLimitSwitch = new DigitalInput(3);
  // public final DigitalInput motor12UpperLimitSwitch = new DigitalInput(5);
  // public final DigitalInput motor11LowerLimitSwitch = new DigitalInput(4);
  // public final DigitalInput motor11UpperLimitSwitch = new DigitalInput(6);

  public final AnalogPotentiometer m_WinchPotRight = new AnalogPotentiometer(1);
  public final AnalogPotentiometer m_WinchPotLeft = new AnalogPotentiometer(0);

  public final AnalogPotentiometer m_WinchStringPotRight = new AnalogPotentiometer(2);
  public final AnalogPotentiometer m_WinchStringPotLeft = new AnalogPotentiometer(3);

  public final TalonFX m_WinchMotorRight = new TalonFX(12, "rio");
  public final TalonFX m_WinchMotorLeft = new TalonFX(11, "rio");

  // public final Servo m_WinchRightServo = new Servo(0);
  // public final Servo m_WinchLeftServo = new Servo(1);

  private VelocityVoltage m_request = new VelocityVoltage(0);
  // public double rightLimitMotor = m_WinchMotorRight.getPosition().getValueAsDouble() + 767.0239;
  // public double leftLimitMotor = m_WinchMotorLeft.getPosition().getValueAsDouble() - 748.43;

  // public boolean winchEnabled = true;

  public ClimbSubsystem() {
    m_WinchMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_WinchMotorRight.setNeutralMode(NeutralModeValue.Brake);

    m_WinchMotorLeft.setInverted(true);
    m_WinchMotorRight.setInverted(false);
  }

  public double applyWinchLimits(boolean left, double speed) {
    if (left) {
      if (m_WinchStringPotLeft.get() > Constants.ClimbConstants.WinchUpperLeftLimit
          && speed > 0.0) {
        return 0.0;
      } else if (m_WinchStringPotLeft.get() < Constants.ClimbConstants.WinchLowerLeftLimit
          && speed < 0.0) {
        return 0.0;
      } else {
        return speed;
      }
    } else {
      if (1 - m_WinchStringPotRight.get() > Constants.ClimbConstants.WinchUpperRightLimit
          && speed > 0.0) {
        return 0.0;
      } else if (1 - m_WinchStringPotRight.get() < Constants.ClimbConstants.WinchLowerRightLimit
          && speed < 0.0) {
        return 0.0;
      } else {
        return speed;
      }
    }
  }

  public void setRightWinchSpeed(double speed) {
    m_WinchMotorRight.set(applyWinchLimits(false, speed));
    // m_WinchMotorRight.set(speed);

    Logger.recordOutput("RightWinchSpeed", speed);
  }

  public void setLeftWinchSpeed(double speed) {
    m_WinchMotorLeft.set(applyWinchLimits(true, speed));
    // m_WinchMotorLeft.set(speed);

    Logger.recordOutput("LeftWinchSpeed", speed);
  }

  public void periodic() {

    // SmartDashboard.putNumber("LeftClimbValue",
    // m_WinchMotorLeft.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Right Potentiometer", m_WinchPotRight.get());
    // SmartDashboard.putNumber("Left Potentiometer", m_WinchPotLeft.get());

    // SmartDashboard.putNumber("Right Potentiometer", 1 - m_WinchStringPotRight.get());
    // SmartDashboard.putNumber("Left Potentiometer", m_WinchStringPotLeft.get());

    Logger.recordOutput("LeftClimbValue", m_WinchMotorLeft.getPosition().getValueAsDouble());
    Logger.recordOutput("RightClimbValue", m_WinchMotorRight.getPosition().getValueAsDouble());
    Logger.recordOutput("LeftPotClimbValue", m_WinchPotLeft.get());
    Logger.recordOutput("RightPotClimbValue", 1 - m_WinchPotRight.get());
  }
}
