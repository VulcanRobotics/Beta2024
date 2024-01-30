package frc.robot.commands;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WinchSubsystem;

public class SetArmPosition extends Command {

  private WinchSubsystem armSubsystem;
  private MotionMagicVoltage m_request;
  private double targetPositionInRotation;

  public SetArmPosition(WinchSubsystem armSubsystem, double targetPositionInRotation) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    var talonFXConfigs = new TalonFXConfiguration();
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.5; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.2; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    armSubsystem.m_ArmMotor1.getConfigurator().apply(talonFXConfigs);
    /*
    armFeedbackController.reset(armSubsystem.getArmEncoder());
    armFeedbackController.setGoal(targetPositionInRotation);
    */
    this.m_request =
        new MotionMagicVoltage(targetPositionInRotation + armSubsystem.getEncoderOffset());
    this.targetPositionInRotation = targetPositionInRotation;
  }

  @Override
  public void execute() {
    armSubsystem.m_ArmMotor1.setControl(
        m_request.withPosition(targetPositionInRotation + armSubsystem.getEncoderOffset()));
  }

  @Override
  public boolean isFinished() {
    return (armSubsystem.getArmEncoder() == 0);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.m_ArmMotor1.setVoltage(0.0);
  }
}
