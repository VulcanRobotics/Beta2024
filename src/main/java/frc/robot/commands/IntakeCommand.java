// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Command that runs the intake and ends when a note has been detected. Pretty strightforward. */
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

    if (shooterSubsystem.intakeSensor.get()) { // TURN BACK ON
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
    // This timer is basically debouncing the intake because otherwise the beambreak will sense a
    // note twice and work incorrectly in auto.
    if (Timer.get() > 0.25 && shooterSubsystem.intakeSensor.get()) {
      return true;
    } else {
      return false;
    }
  }
}
