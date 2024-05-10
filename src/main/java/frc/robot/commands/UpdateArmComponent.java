// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** An example command that uses an example subsystem. */
public class UpdateArmComponent extends Command {
  ArmSubsystem armSubsystem;
  Drive drive;
  DoubleSupplier supplier;

  public UpdateArmComponent(ArmSubsystem arm, Drive drive) {
    this.armSubsystem = arm;
    this.drive = drive;
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput(
        "Arm Component",
        new Pose3d(
            0.1,
            -0.39,
            0.85,
            new Rotation3d(
                Units.degreesToRadians(180),
                Units.degreesToRadians(0),
                Units.degreesToRadians(90))));
  }
  // Math.toRadians(armSubsystem.getArmEncoder())
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
