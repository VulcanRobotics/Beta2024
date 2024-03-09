// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  boolean stopShooter;

  public IntakeCommand(ShooterSubsystem shooter, boolean stop) {
    this.shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
    stopShooter = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.intakeSensor.get()) { // TURN BACK ON
      shooterSubsystem.SetIntake(0);
      shooterSubsystem.SetFeeder(0);
    } else {
      shooterSubsystem.SetIntake(0.75f);
      shooterSubsystem.SetFeeder(0.25f);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.SetIntake(0);
    shooterSubsystem.SetFeeder(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.intakeSensor.get();
  }
}
