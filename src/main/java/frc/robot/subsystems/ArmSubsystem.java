package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.*;

public class ArmSubsystem extends SubsystemBase {
  public final TalonFX m_ArmMotor1 = new TalonFX(15, "rio");
  public final TalonFX m_ArmMotor2 = new TalonFX(14, "rio");
  private Follower m_follow = new Follower(ArmConstants.kGuideMotorPort, true);

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
        ArmConstants.kArmTargetJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_ArmMotor1.getConfigurator().apply(talonFXConfigs);
    m_ArmMotor2.getConfigurator().apply(talonFXConfigs);
    TalonUtil.setBrakeMode(m_ArmMotor1);
    TalonUtil.setBrakeMode(m_ArmMotor2);
    // Maybe put these booleans in constants
    m_ArmMotor1.setInverted(false);
    m_ArmMotor2.setInverted(true);
  }

  public double getArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble();
  }

  /**
   * This method sets the integrated encoder of TalonFX to 0. Without an absolute encoder present,
   * this should be done before the start of every match.
   */
  public void zeroArmEncoder() {
    m_ArmMotor1.setPosition(0.0);
  }

  public void setArmSpeed(double speed) {
    speed = speed *= 0.6;
    m_ArmMotor1.set(speed);
    m_ArmMotor2.setControl(m_follow);
  }

  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Value", m_ArmMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putString("Brake Mode", m_ArmMotor1.getConfigurator().toString());
  }
}
