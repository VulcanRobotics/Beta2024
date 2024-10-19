package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.util.*;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Contains all arm funcionality */
public class ArmSubsystem extends SubsystemBase {
  public final TalonFX m_ArmMotor1 = new TalonFX(14, "rio");
  public final TalonFX m_ArmMotor2 = new TalonFX(15, "rio");

  public final CANcoder m_ArmEncoder = new CANcoder(13, "rio");

  private Follower m_follow = new Follower(14, true);

  public boolean inAmpPosition = false;

  public final double topLimit = 93; // 87.5
  public final double botLimit = 0.0;

  // This state is only ever used in experimental autos.
  private ArmStates currentState = ArmStates.DRIVER;
  // Supplier is also only used in those autos.
  private Optional<DoubleSupplier> angleSupplier;

  public ArmSubsystem() {
    var talonFXConfigs = new TalonFXConfiguration();

    // var cancoderConfig = new CANcoderConfiguration();
    // cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    // m_ArmEncoder.getConfigurator().apply(canZcoderConfig);

    // m_ArmMotor2.setControl(m_follow);

    // set slot 0 gains

    // Set up PID and Feedforward config for the motors. Calculated bases on experimental values.
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = ArmConstants.kArmKS;
    slot0Configs.kV = ArmConstants.kArmKV;
    slot0Configs.kA = ArmConstants.kArmKA;
    slot0Configs.kP = ArmConstants.kArmKP;
    slot0Configs.kI = ArmConstants.kArmKI;
    slot0Configs.kD = ArmConstants.kArmKD;

    // set Motion Magic settings; basically a motion profile that is very easy to use
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        ArmConstants.kArmTargetVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.kArmTargetAcceleration;
    motionMagicConfigs.MotionMagicJerk = ArmConstants.kArmTargetJerk;

    m_ArmMotor1.getConfigurator().apply(talonFXConfigs);
    m_ArmMotor2.getConfigurator().apply(talonFXConfigs);
    TalonUtil.setBrakeMode(m_ArmMotor1);
    TalonUtil.setBrakeMode(m_ArmMotor2);
    // Maybe put these booleans in constants
    m_ArmMotor1.setInverted(true);
    m_ArmMotor2.setInverted(true);

    // Ensure that arm is zeroed
    calibrateTalonEncoder();
  }

  /** Reads the arm angle in degrees (0 - 90) */
  public double getArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble()
        * Constants.ArmConstants.kMotorEncoderToDegrees;
  }

  public double getRawArmEncoder() {
    return m_ArmMotor1.getPosition().getValueAsDouble();
  }

  /** Used to constantly update the angle (0 - 90) of the arm with a supplier. */
  public void setArmPosition(DoubleSupplier supplier) {
    double targetPositionInDegrees = supplier.getAsDouble();
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPositionInRotation);
    m_ArmMotor1.setControl(m_request);
    m_ArmMotor2.setControl(m_follow);
  }

  /** Used to set the arm to a certain constant angle (0 - 90). */
  public void setArmPosition(double angle) {
    double targetPositionInDegrees = angle;
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPositionInRotation);
    m_ArmMotor1.setControl(m_request);
    m_ArmMotor2.setControl(m_follow);
  }

  // Used once in robot container
  public void configArmAngleSupplier(DoubleSupplier supplier) {
    this.angleSupplier = Optional.of(supplier);
  }

  // Only used in some autos
  public void setArmState(ArmConstants.ArmStates state) {
    this.currentState = state;
  }

  /**
   * This method sets the integrated encoder of TalonFX to 0. Without an absolute encoder present,
   * this should be done before the start of every match. However, we currently have a CANCoder, so
   * there is no need to use this function.
   */
  public void zeroArmEncoder() {
    m_ArmMotor1.setPosition(0.0);
  }

  /**
   * This method uses the absolute encoder to determine where the arm is and set the internal motor
   * encoder according to that position.
   */
  public void calibrateTalonEncoder() {
    double delta =
        ArmConstants.kCanCoderZeroPosition - m_ArmEncoder.getAbsolutePosition().getValueAsDouble();
    m_ArmMotor1.setPosition(delta * ArmConstants.kCanCoderToArmMotorRatio);
  }

  /** This is pretty janky ngl */
  public void setArmSpeed(double speed) {
    speed = speed *= 0.6;
    speed = applyLimits(speed);
    m_ArmMotor1.set(speed);
    m_ArmMotor2.setControl(m_follow);
  }

  // Soft limits for both speed and position
  public double applyLimits(double speed) {
    if (getArmEncoder() < botLimit && speed < 0) { // getArmEncoder() - 0.5f <= 0 && speed < 0
      return 0;
    } else if (getArmEncoder() > topLimit && speed > 0) { // getArmEncoder() + 1f >= 77 && speed > 0
      return 0;
    }
    return (speed);
  }

  public void periodic() {

    // Used for determining shooter velocity
    if (getArmEncoder() > 70.0) {
      inAmpPosition = true;
    } else {
      inAmpPosition = false;
    }

    // This is specifically used for auto in order to control the arm throughout the routing.
    if (DriverStation.isAutonomous()) {
      switch (currentState) {
        case SHOOTING:
          if (angleSupplier.isPresent()) setArmPosition(angleSupplier.get());
          break;

        case INTAKE:
          setArmPosition(ArmConstants.kArmPoseIntake);
          break;

        case AMP:
          setArmPosition(ArmConstants.kArmPoseAmp);
          break;

        default:
          break;
      }
    }

    Logger.recordOutput("Arm/Arm encoder", m_ArmEncoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("Arm/Arm Angle (0-90)", getArmEncoder());
    Logger.recordOutput("Arm/Arm motor encoder", m_ArmMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Arm Angle (0-90)", getArmEncoder());
  }
}
