// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
            drive.getPose().getX(),
            drive.getPose().getY(),
            0.8,
            new Rotation3d(
                0,
                2 * Math.toRadians(armSubsystem.getArmEncoder()),
                drive.getRotation().getRadians())));
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}