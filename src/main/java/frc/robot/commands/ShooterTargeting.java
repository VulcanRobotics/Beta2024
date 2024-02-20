package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

// plan to place code for autoshoot here; break it up into multiple commands (e.g. arm, shooter,
// turning)
public class ShooterTargeting extends Command {
  ShooterSubsystem shooter;
  ArmSubsystem arm;
  Drive drive;

  public ShooterTargeting(Drive drive, ArmSubsystem arm, ShooterSubsystem shooter) {
    addRequirements(arm, drive, shooter);
    this.arm = arm;
    this.drive = drive;
    this.shooter = shooter;
  }
}
