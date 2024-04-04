// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class ClimbSetupCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  ClimbSubsystem climb;
  ArmSubsystem arm;
  Drive drive;
  DoubleSupplier supplier;

  public ClimbSetupCommand(ClimbSubsystem climbSubsystem, ArmSubsystem armSubsystem) {
    this.climb = climbSubsystem;
    this.arm = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setLeftWinchSpeed(1.0);
    climb.setRightWinchSpeed(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setTrapMotorPosition(145);
    new SetArmPosition(arm, () -> ArmConstants.kArmPoseAmp);
  }

  @Override
  public void end(boolean interrupted) {
    climb.setLeftWinchSpeed(0.0);
    climb.setRightWinchSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
