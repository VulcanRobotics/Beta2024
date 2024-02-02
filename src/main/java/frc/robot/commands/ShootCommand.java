// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  ShooterSubsystem shooterSubsystem;

  boolean fullPower;

  public ShootCommand(ShooterSubsystem shooter, boolean Max) {
    this.shooterSubsystem = shooter;
    fullPower = Max;
  }

  private double shootSpeed = 90;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fullPower == true) {
      shooterSubsystem.savedShootSpeed = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.SetShooter(shooterSubsystem.savedShootSpeed);
    if (shooterSubsystem.getAverageShootSpeed() >= shootSpeed) {
      shooterSubsystem.SetFeeder(1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.SetShooter(0);

    shooterSubsystem.SetFeeder(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
