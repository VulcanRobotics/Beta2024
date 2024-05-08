// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * This command spins the shooter motors to a desired exit velocity. This desired velocity changes
 * depending on the position of the robot on the field: - if the robot is within our wing, the
 * motors will spin up close to max - if the robot is within the midzone (between both wings), the
 * motors will spin slower to properly shuttle the note without launching it out. - if the robot's
 * arm is up in Amp position, the motors will spin very slowly to drop the note in
 */
public class RevCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  ArmSubsystem armSubsystem;
  Drive drive;
  DoubleSupplier supplier;

  public RevCommand(ShooterSubsystem shooter, ArmSubsystem arm, Drive drive) {
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
    double velocity =
        (armSubsystem.inAmpPosition)
            ? Constants.ShooterConstants.kShooterTargetVelocity * 0.2
            : Constants.ShooterConstants
                .kShooterTargetVelocity; // If the robot's arm is in Amp Position, decrease default
    // velocity by 80%

    if (drive.getPose().getX() < 10.75
        && drive.getPose().getX() > 5.85) { // If the robot is in the midfield, set velocity to 50
      velocity = 50;
    }

    shooterSubsystem.setShooterVelocity(velocity); // After parameters, set shooter velocity

    if (shooterSubsystem.getAverageShootSpeed()
        >= (velocity)) { // Used to help other systems know whether the shooter is reved up all the
      // way
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
