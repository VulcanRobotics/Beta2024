// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  ArmSubsystem armSubsystem;
  Drive drive;

  public ShootCommand(ShooterSubsystem shooter, ArmSubsystem arm, Drive drive) {
    this.shooterSubsystem = shooter;
    this.armSubsystem = arm;
    this.drive = drive;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getAverageShootSpeed()
            > (Constants.ShooterConstants.kShooterTargetVelocity - 10.0)
        || armSubsystem.inAmpPosition
        || (drive.getPose().getX() < 10.75
            && drive.getPose().getX()
                > 5.85)) { // Make this conditional on whether the shooter is up to speed
      shooterSubsystem.SetFeeder(1.0f);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.SetFeeder(0);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooterSubsystem.intakeSensor.get();
  }
}
