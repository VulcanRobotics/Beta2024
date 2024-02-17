package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootToggle extends Command {
  ShooterSubsystem shooterSubsystem;

  public ShootToggle(ShooterSubsystem shooter) {
    this.shooterSubsystem = shooter;
  }

  @Override
  public void execute() {
    if (shooterSubsystem.toggleShooter == true) {
      shooterSubsystem.SetShooter(0);
    }
    shooterSubsystem.toggleShooter = !shooterSubsystem.toggleShooter;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
