// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

/** An example command that uses an example subsystem. */
public class RevCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  ArmSubsystem armSubsystem;
  DoubleSupplier supplier;

  public RevCommand(ShooterSubsystem shooter, ArmSubsystem arm) {
    this.shooterSubsystem = shooter;
    this.armSubsystem = arm;
  }

  private double shootSpeed = 75;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if (fullPower == true) {
      shooterSubsystem.savedShootSpeed = 1;
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity =
        (armSubsystem.inAmpPosition)
            ? Constants.ShooterConstants.kShooterTargetVelocity * 0.3
            : Constants.ShooterConstants.kShooterTargetVelocity;
    shooterSubsystem.setShooterVelocity(velocity); // shooterSubsystem.savedShootSpeed);
    if (shooterSubsystem.getAverageShootSpeed() >= (velocity)) {
      shooterSubsystem.upToSpeed = true;
    } else {
      shooterSubsystem.upToSpeed = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.SetShooter(0.0);
    shooterSubsystem.upToSpeed = false;
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
