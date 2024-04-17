package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  public final TalonFX m_ArmMotor1 = new TalonFX(15, "rio");
  public final TalonFX m_ArmMotor2 = new TalonFX(14, "rio");

  public final CANcoder m_ArmEncoder = new CANcoder(13, "rio");

  private Follower m_follow = new Follower(ArmConstants.kGuideMotorPort, true);

  public boolean inAmpPosition = false;

  public final double topLimit = 93; // 87.5
  public final double botLimit = 0.0;

  public ArmSubsystem() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ArmConstants.kArmKS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = ArmConstants.kArmKV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = ArmConstants.kArmKA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        ArmConstants.kArmKP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = ArmConstants.kArmKI; // no output for integrated error
    slot0Configs.kD = ArmConstants.kArmKD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        ArmConstants.kArmTargetVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        ArmConstants.kArmTargetAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk =
        ArmConstants.kArmTargetJerk; // Target jerk of 1600 rps/s/s (0.1 seconds

    m_ArmMotor1.getConfigurator().apply(talonFXConfigs);
    m_ArmMotor2.getConfigurator().apply(talonFXConfigs);
    TalonUtil.setBrakeMode(m_ArmMotor1);
    TalonUtil.setBrakeMode(m_ArmMotor2);
    // Maybe put these booleans in constants
    m_ArmMotor1.setInverted(false);
    m_ArmMotor2.setInverted(true);

    // Ensure that arm is zeroed
    calibrateTalonEncoder();
  }

  public double getArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble()
        * Constants.ArmConstants.kMotorEncoderToDegrees;
  }

  public double getRawArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble();
  }

  public void setArmPosition(DoubleSupplier supplier) {
    double targetPositionInDegrees = supplier.getAsDouble();
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPositionInRotation);
    m_ArmMotor1.setControl(m_request);
    m_ArmMotor2.setControl(m_follow);
  }

  /**
   * This method sets the integrated encoder of TalonFX to 0. Without an absolute encoder present,
   * this should be done before the start of every match.
   */
  public void zeroArmEncoder() {
    m_ArmMotor1.setPosition(0.0);
  }

  // This method ensures that the the Talon's 'zero' position is equivalent to its intake state
  public void calibrateTalonEncoder() {
    double delta =
        ArmConstants.kCanCoderZeroPosition - m_ArmEncoder.getAbsolutePosition().getValueAsDouble();
    m_ArmMotor1.setPosition(delta * ArmConstants.kCanCoderToArmMotorRatio);
  }

  public void setArmSpeed(double speed) {
    speed = speed *= 0.6;
    speed = applyLimits(speed);
    m_ArmMotor1.set(speed);
    m_ArmMotor2.setControl(m_follow);
  }

  public double applyLimits(double speed) {
    if (getArmEncoder() < botLimit && speed < 0) { // getArmEncoder() - 0.5f <= 0 && speed < 0
      return 0;
    } else if (getArmEncoder() > topLimit && speed > 0) { // getArmEncoder() + 1f >= 77 && speed > 0
      return 0;
    }
    return (speed);
  }

  public void periodic() {

    if (getArmEncoder() > 70.0) {
      inAmpPosition = true;
    } else {
      inAmpPosition = false;
    }

    Logger.recordOutput("Arm/Arm encoder", m_ArmEncoder.getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("Arm Encoder",
    // m_ArmEncoder.getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("Arm motor encoder", m_ArmMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm Angle (0-90)", getArmEncoder());

    Logger.recordOutput("Arm/Arm encoder", m_ArmEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("Arm/Arm Angle (0-90)", getArmEncoder());
    Logger.recordOutput("Arm/Arm motor encoder", m_ArmMotor1.getPosition().getValueAsDouble());
  }
}
