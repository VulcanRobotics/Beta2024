// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This turns on the intake rollers indefinatly until the robot has the note for at least 0.25
 * seconds.
 */
public class IntakeCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  Timer Timer = new Timer();

  public IntakeCommand(ShooterSubsystem shooter) {
    this.shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Timer.reset();
    Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shooterSubsystem.intakeSensor
        .get()) { // If you have the note, stop intaking. If you don't keep intaking while
      // constantly restarting clock.
      shooterSubsystem.SetIntake(0);
      shooterSubsystem.SetFeeder(0);
    } else {
      shooterSubsystem.SetIntake(0.75f);
      shooterSubsystem.SetFeeder(0.25f);
      Timer.reset();
      Timer.start();
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
    if (Timer.get() > 0.25
        && shooterSubsystem.intakeSensor
            .get()) { // If clock higher than 0.25 seconds and we still have note, stop command
      return true;
    } else {
      return false;
    }
  }
}
