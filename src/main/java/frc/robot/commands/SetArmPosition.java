package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmIO;
import frc.robot.subsystems.ArmIOInputsAutoLogged;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

/** Uses motion magic on the two kraken motors to smoothly move the arm to its desired position */
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

  private ArmIO io;
  private ArmIOInputsAutoLogged inputs;

  public SetArmPosition(ArmSubsystem armSubsystem, DoubleSupplier doubleSupplier) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.supplier = doubleSupplier;
    this.inputs = armSubsystem.inputs;
    this.io = armSubsystem.io;

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

    io.setArmMotor1Request(targetPositionInRotation);
    io.setArmMotor2Follow();
  }

  @Override
  public boolean isFinished() {
    double delta = goalPosition - inputs.armMotor1Pos;
    if (Math.abs(delta) < ArmConstants.kArmPIDTolerance) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    io.setArmMotor1Speed(0.0); // This should just stop the motor in its tracks.
  }
}
