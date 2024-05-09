package frc.robot.subsystems;

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

  // The string pots that we had to get rid of due to them breaking constantly. //
  // public final AnalogPotentiometer m_WinchPotRight = new AnalogPotentiometer(1);
  // public final AnalogPotentiometer m_WinchPotLeft = new AnalogPotentiometer(0);
  // public final AnalogPotentiometer m_WinchStringPotRight = new AnalogPotentiometer(2);
  // public final AnalogPotentiometer m_WinchStringPotLeft = new AnalogPotentiometer(3);

  public final TalonFX m_WinchMotorRight = new TalonFX(12, "rio");
  public final TalonFX m_WinchMotorLeft = new TalonFX(11, "rio");

  public final AnalogPotentiometer m_TrapPot = new AnalogPotentiometer(2);

  public final CANSparkMax m_TrapMotor =
      new CANSparkMax(ClimbConstants.kTrapMotorPort, MotorType.kBrushless);

  public final double trapMotorStartingPos = m_TrapMotor.getEncoder().getPosition();

  public ClimbSubsystem() {
    m_WinchMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    m_WinchMotorRight.setNeutralMode(NeutralModeValue.Brake);

    m_WinchMotorLeft.setInverted(true);
    m_WinchMotorRight.setInverted(false);
  }

  // The function below made the climb motors stop when the string pots reached a desried distace,
  // //
  // but the string pots broke multiple times during competition so this is commented out. //

  // public double applyWinchLimits(boolean left, double speed) {
  //   if (left) {
  //     if (m_WinchStringPotLeft.get() > Constants.ClimbConstants.WinchUpperLeftLimit
  //         && speed > 0.0) {
  //       return 0.0;
  //     } else if (m_WinchStringPotLeft.get() < Constants.ClimbConstants.WinchLowerLeftLimit
  //         && speed < 0.0) {
  //       return 0.0;
  //     } else {
  //       return speed;
  //     }
  //   } else {
  //     if (1 - m_WinchStringPotRight.get() > Constants.ClimbConstants.WinchUpperRightLimit
  //         && speed > 0.0) {
  //       return 0.0;
  //     } else if (1 - m_WinchStringPotRight.get() < Constants.ClimbConstants.WinchLowerRightLimit
  //         && speed < 0.0) {
  //       return 0.0;
  //     } else {
  //       return speed;
  //     }
  //   }
  // }

  /**
   * Uses the trap string pot to prevent the bar from going over the limit. Had to stop using it
   * because that string pot broke too.
   */
  public double applyTrapLimits(double speed) {
    if (m_TrapPot.get() > Constants.ClimbConstants.TrapBotLimit
        || m_TrapPot.get() < Constants.ClimbConstants.TrapTopLimit) {
      return 0.0;
    } else {
      return speed;
    }
  }

  /**
   * Uses the relative encoder on the trap motor to automatically set the bar to the desired encoder
   * value.
   */
  public void setTrapMotorPosition(double targetPosition) {
    double currentPosition = m_TrapMotor.getEncoder().getPosition();

    if ((targetPosition - currentPosition) > 0.1f) {
      setTrapSpeed(0.25f);
    } else if ((targetPosition - currentPosition) < -0.1f) {
      setTrapSpeed(-0.25f);
    }
  }

  /** Exactly what the function title entails. */
  public void setTrapSpeed(double speed) {
    // m_TrapMotor.set(applyTrapLimits(speed));
    m_TrapMotor.set(speed);
  }

  /** Exactly what the function title entails. */
  public void setRightWinchSpeed(double speed) {
    // m_WinchMotorRight.set(applyWinchLimits(false, speed));
    m_WinchMotorRight.set(speed);

    Logger.recordOutput("Climb/RightWinchSpeed", speed);
  }

  /** Exactly what the function title entails. */
  public void setLeftWinchSpeed(double speed) {
    // m_WinchMotorLeft.set(applyWinchLimits(true, speed));
    m_WinchMotorLeft.set(speed);

    Logger.recordOutput("Climb/LeftWinchSpeed", speed);
  }

  public void periodic() {

    // SmartDashboard.putNumber("Right Potentiometer", 1 - m_WinchStringPotRight.get());
    // SmartDashboard.putNumber("Left Potentiometer", m_WinchStringPotLeft.get());

    SmartDashboard.putNumber("Trap Motor Position", m_TrapPot.get());
    SmartDashboard.putNumber("Trap Speed", m_TrapMotor.get());

    Logger.recordOutput("Climb/LeftClimbValue", m_WinchMotorLeft.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Climb/RightClimbValue", m_WinchMotorRight.getPosition().getValueAsDouble());
    // Logger.recordOutput("Climb/LeftPotClimbValue", m_WinchPotLeft.get());
    // Logger.recordOutput("Climb/RightPotClimbValue", 1 - m_WinchPotRight.get());
  }
}
