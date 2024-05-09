package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

public class ArmSubsystem extends SubsystemBase {

  private Follower m_follow = new Follower(ArmConstants.kGuideMotorPort, true);

  public boolean inAmpPosition = false;

  public final double topLimit = 93; // 87.5
  public final double botLimit = 0.0;

  private ArmStates currentState = ArmStates.DRIVER;
  private Optional<DoubleSupplier> angleSupplier;
  public static Pose3d armComponent = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  public static double armAngle = 0.0;

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem() {

    if (Constants.currentMode == Constants.Mode.SIM) {
      io = new ArmIOSim();
    } else {
      io = new ArmIOTalonFX();
    }

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    // Set up PID and Feedforward config for the motors.
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.6; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.05; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        ArmConstants.kArmTargetVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        ArmConstants.kArmTargetAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk =
        ArmConstants.kArmTargetJerk; // Target jerk of 1600 rps/s/s (0.1 seconds

    // Maybe put these booleans in constants

    // Ensure that arm is zeroed
    calibrateTalonEncoder();
  }

  public double getArmEncoder() {
    return inputs.armMotor1Pos * Constants.ArmConstants.kMotorEncoderToDegrees;
  }

  public double getRawArmEncoder() {
    return inputs.armMotor1Pos;
  }

  /** Used to constantly update the angle (0 - 90) of the arm with a supplier. */
  public void setArmPosition(DoubleSupplier supplier) {
    double targetPositionInDegrees = supplier.getAsDouble();
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPositionInRotation);
    io.setArmMotor1Request(m_request);
    io.setArmMotor2Follow(m_follow);
  }

  /** Used to set the arm to a certain constant angle (0 - 90). */
  public void setArmPosition(double angle) {
    double targetPositionInDegrees = angle;
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPositionInRotation);
    io.setArmMotor1Request(m_request);
    io.setArmMotor2Follow(m_follow);
  }

  public void configArmAngleSupplier(DoubleSupplier supplier) {
    this.angleSupplier = Optional.of(supplier);
  }

  public void setArmState(ArmConstants.ArmStates state) {
    this.currentState = state;
  }

  /**
   * This method sets the integrated encoder of TalonFX to 0. Without an absolute encoder present,
   * this should be done before the start of every match. However, we currently have a CANCoder, so
   * there is no need to use this function.
   */
  public void zeroArmEncoder() {
    // m_ArmMotor1.setPosition(0.0);
  }

  /**
   * This method uses the absolute encoder to determine where the arm is and set the internal motor
   * encoder according to that position.
   */
  public void calibrateTalonEncoder() {
    double delta = ArmConstants.kCanCoderZeroPosition - inputs.armCancoderPos;
    io.setArmMotor1Pos(delta * ArmConstants.kCanCoderToArmMotorRatio);
  }

  /** This is pretty janky ngl */
  public void setArmSpeed(double speed) {
    speed = speed *= 0.6;
    speed = applyLimits(speed);
    io.setArmMotor1Speed(speed);
    io.setArmMotor2Follow(m_follow);
  }

  /**
   * Stops the current speed value from breaking the arm if the arm in past the bottom or top
   * limits.
   */
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
    Logger.recordOutput("Arm/Arm encoder", inputs.armCancoderPos);
    Logger.recordOutput("Arm/Arm Angle (0-90)", getArmEncoder());
    Logger.recordOutput("Arm/Arm Radians", Math.toRadians(getArmEncoder()));
    Logger.recordOutput("Arm/Arm motor encoder", inputs.armMotor1Pos);
    SmartDashboard.putNumber("Arm Angle (0-90)", getArmEncoder());
  }
}
