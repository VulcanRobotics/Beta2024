package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

// ** Uses motion magic on the two kraken motors to smoothly move the arm to its desired position */
public class SetArmPosition extends Command {

  private double goalPosition;
  private DoubleSupplier supplier;
  private ArmSubsystem armSubsystem;
  // Motion Magic is basically upgraded PID and motion profiling from Phoenix 6; PID values in
  // constants.
  private MotionMagicVoltage m_request;
  // This control request will allow the second motor on the arm to match the one that is guiding.
  private Follower m_follow =
      new Follower(ArmConstants.kGuideMotorPort, ArmConstants.kArm2Inverted);

  public SetArmPosition(ArmSubsystem armSubsystem, DoubleSupplier doubleSupplier) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.supplier = doubleSupplier;

    // The following might be redundant code
    double targetPositionInDegrees = doubleSupplier.getAsDouble();

    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);

    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    this.m_request = new MotionMagicVoltage(targetPositionInRotation);
    this.goalPosition = targetPositionInRotation;
  }

  @Override
  public void execute() {
    double targetPositionInDegrees = supplier.getAsDouble();
    targetPositionInDegrees = MathUtil.clamp(targetPositionInDegrees, 0, 90);
    double targetPositionInRotation =
        targetPositionInDegrees * 1 / Constants.ArmConstants.kMotorEncoderToDegrees;
    this.m_request = m_request.withPosition(targetPositionInRotation);
    this.goalPosition = targetPositionInRotation;

    armSubsystem.m_ArmMotor1.setControl(m_request);
    armSubsystem.m_ArmMotor2.setControl(m_follow);
  }

  @Override
  public boolean isFinished() {
    double delta = goalPosition - armSubsystem.m_ArmMotor1.getPosition().getValueAsDouble();
    if (Math.abs(delta) < ArmConstants.kArmPIDTolerance) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.m_ArmMotor1.stopMotor(); // This should just stop the motor in its tracks.
  }
}
