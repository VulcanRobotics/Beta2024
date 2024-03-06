package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command winchDrive(
      ClimbSubsystem climbSubsystem, DoubleSupplier yAxis, DoubleSupplier xAxis) {
    double DEADBAND = 0.5;

    return Commands.run(
        () -> {
          double speed = yAxis.getAsDouble();
          double difference = xAxis.getAsDouble();

          if (Math.abs(speed) < 0.1) {
            speed = 0.0;
          }

          if (difference > DEADBAND) {
            climbSubsystem.setRightWinchSpeed(speed);
          } else if (difference < -DEADBAND) {
            climbSubsystem.setLeftWinchSpeed(speed);
          } else {
            climbSubsystem.setRightWinchSpeed(0);
            climbSubsystem.setLeftWinchSpeed(0);
          }

          // speed *= speed;
        },
        climbSubsystem);
  }

  public static Command raiseToLowChain(ClimbSubsystem climb) {
    return Commands.run(
        () -> {
          if (climb.m_WinchPotRight.get()
              < (Constants.ClimbConstants.WinchUpperRightLimit
                  - Constants.ClimbConstants.kRightTopDistanceFromChain)) {
            climb.setRightWinchSpeed(1.0);
          } else {
            climb.setRightWinchSpeed(0.0);
          }

          if (climb.m_WinchPotLeft.get()
              < (Constants.ClimbConstants.WinchUpperLeftLimit
                  + Constants.ClimbConstants.kLeftTopDistanceFromChain)) {
            climb.setLeftWinchSpeed(1.0);
          } else {
            climb.setLeftWinchSpeed(0.0);
          }
        },
        climb);
  }
}
