// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** An example command that uses an example subsystem. */
public class LockWinchCommand extends Command {
  ClimbSubsystem climbSubsystem;

  boolean toggle = false;

  public LockWinchCommand(ClimbSubsystem climb) {
    this.climbSubsystem = climb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    toggle = !toggle;

    climbSubsystem.winchEnabled = !toggle;
    climbSubsystem.setServoLock(toggle);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
