package frc.robot.commands;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPosition extends Command {

  private ArmSubsystem armSubsystem;
  // Motion Magic is basically upgraded PID and motion profiling from Phoenix 6; PID values in
  // constants.
  private MotionMagicVoltage m_request;
  // This control request will allow the second motor on the arm to match the one that is guiding.
  private Follower m_follow =
      new Follower(ArmConstants.kGuideMotorPort, ArmConstants.kArm2Inverted);

  // public SetArmPosition(ArmSubsystem armSubsystem, double targetPositionInRotation) {
  //   addRequirements(armSubsystem);
  //   this.armSubsystem = armSubsystem;
  //   this.m_request = new MotionMagicVoltage(targetPositionInRotation);
  // }

  public SetArmPosition(ArmSubsystem armSubsystem, double targetPositionInDegrees) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    double targetPositionInRotation =
        targetPositionInDegrees / 360 * 320; // 10:1 (this will become 9:1), 6:1, 80:15
    this.m_request = new MotionMagicVoltage(targetPositionInRotation);
  }

  @Override
  public void execute() {
    armSubsystem.m_ArmMotor1.setControl(m_request);
    armSubsystem.m_ArmMotor2.setControl(m_follow);
  }

  @Override
  public boolean isFinished() {
    return false; // Find a better condition for this to be finished.
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.m_ArmMotor1.stopMotor(); // This should just stop the motor in its tracks.
  }
}
