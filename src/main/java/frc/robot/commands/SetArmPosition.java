package frc.robot.commands;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPosition extends Command {

  private ArmSubsystem armSubsystem;
  private MotionMagicVoltage m_request;

  public SetArmPosition(ArmSubsystem armSubsystem, double targetPositionInRotation) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.m_request = new MotionMagicVoltage(targetPositionInRotation);
  }

  @Override
  public void execute() {
    armSubsystem.m_ArmMotor1.setControl(m_request);
  }

  @Override
  public boolean isFinished() {
    return (armSubsystem.getArmEncoder() == m_request.Position);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.m_ArmMotor1.setVoltage(0.0);
  }
}
