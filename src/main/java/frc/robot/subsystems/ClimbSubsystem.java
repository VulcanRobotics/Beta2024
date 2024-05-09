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

/** Simple controls for our climbers */
public class ClimbSubsystem extends SubsystemBase {

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

  public double applyTrapLimits(double speed) {
    if (m_TrapPot.get() > Constants.ClimbConstants.TrapBotLimit
        || m_TrapPot.get() < Constants.ClimbConstants.TrapTopLimit) {
      return 0.0;
    } else {
      return speed;
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
    // m_TrapMotor.set(applyTrapLimits(speed));
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

    SmartDashboard.putNumber("Trap Motor Position", m_TrapPot.get());
    SmartDashboard.putNumber("Trap Speed", m_TrapMotor.get());

    Logger.recordOutput("Climb/LeftClimbValue", m_WinchMotorLeft.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Climb/RightClimbValue", m_WinchMotorRight.getPosition().getValueAsDouble());
  }
}
