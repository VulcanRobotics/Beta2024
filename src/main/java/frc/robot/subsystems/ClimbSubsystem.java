package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  public final AnalogPotentiometer m_WinchPotRight = new AnalogPotentiometer(1);
  public final AnalogPotentiometer m_WinchPotLeft = new AnalogPotentiometer(0);

  public final AnalogPotentiometer m_WinchStringPotRight = new AnalogPotentiometer(2);
  public final AnalogPotentiometer m_WinchStringPotLeft = new AnalogPotentiometer(3);

  public final TalonFX m_WinchMotorRight = new TalonFX(12, "rio");
  public final TalonFX m_WinchMotorLeft = new TalonFX(11, "rio");

  public final CANSparkMax m_TrapMotor =
      new CANSparkMax(ClimbConstants.kTrapMotorPort, MotorType.kBrushless);

  public final double trapMotorStartingPos = m_TrapMotor.getEncoder().getPosition();

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

  public void setTrapMotorPosition(double targetPosition) {
    double currentPosition = m_TrapMotor.getEncoder().getPosition();

    if ((targetPosition - currentPosition) > 0.1f) {
      setTrapSpeed(0.25f);
    } else if ((targetPosition - currentPosition) < -0.1f) {
      setTrapSpeed(-0.25f);
    }
  }

  public void setTrapSpeed(double speed) {
    if (m_TrapMotor.getEncoder().getPosition() <= 0 && speed < 0) {
      speed = 0;
    } else if (m_TrapMotor.getEncoder().getPosition() >= ClimbConstants.kTrapMotorUpperOffset
        && speed > 0) {
      speed = 0;
    }

    m_TrapMotor.set(speed);
  }

  public void setRightWinchSpeed(double speed) {
    // m_WinchMotorRight.set(applyWinchLimits(false, speed));
    m_WinchMotorRight.set(speed);

    Logger.recordOutput("Climb/RightWinchSpeed", speed);
  }

  public void setLeftWinchSpeed(double speed) {
    // m_WinchMotorLeft.set(applyWinchLimits(true, speed));
    m_WinchMotorLeft.set(speed);

    Logger.recordOutput("Climb/LeftWinchSpeed", speed);
  }

  public void periodic() {

    SmartDashboard.putNumber("Right Potentiometer", 1 - m_WinchStringPotRight.get());
    SmartDashboard.putNumber("Left Potentiometer", m_WinchStringPotLeft.get());

    SmartDashboard.putNumber("Trap Motor Position", m_TrapMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Trap Speed", m_TrapMotor.get());

    Logger.recordOutput("Climb/LeftClimbValue", m_WinchMotorLeft.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Climb/RightClimbValue", m_WinchMotorRight.getPosition().getValueAsDouble());
    Logger.recordOutput("Climb/LeftPotClimbValue", m_WinchPotLeft.get());
    Logger.recordOutput("Climb/RightPotClimbValue", 1 - m_WinchPotRight.get());
  }
}
