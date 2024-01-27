package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class SetArmPosition extends Command {

  private WinchSubsystem armSubsystem;
  private ProfiledPIDController armFeedbackController =
      new ProfiledPIDController(0.045, 0.0, 0.0, new TrapezoidProfile.Constraints(7.0, 5.0));

  public SetArmPosition(WinchSubsystem armSubsystem, double targetPositionInRotation) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    armFeedbackController.reset(armSubsystem.getArmEncoder());
    armFeedbackController.setGoal(targetPositionInRotation);
  }

  @Override
  public void execute() {
    armSubsystem.m_ArmMotor1.set(armFeedbackController.calculate(armSubsystem.getArmEncoder()));
  }

  @Override
  public boolean isFinished() {
    return armFeedbackController.atGoal() ? true : false;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.m_ArmMotor1.set(0.0);
  }
}
