// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Command that simply spits out the note by reversing intake motors */
public class DispenseCommand extends Command {
  ShooterSubsystem shooterSubsystem;

  public DispenseCommand(ShooterSubsystem shooter) {
    this.shooterSubsystem = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.SetIntake(-0.25f);
    shooterSubsystem.SetFeeder(-0.25f);
    shooterSubsystem.SetShooter(-0.25f);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.SetIntake(0);
    shooterSubsystem.SetFeeder(0);
    shooterSubsystem.SetShooter(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
