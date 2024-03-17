package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command winchDrive(
      ClimbSubsystem climbSubsystem, DoubleSupplier yAxis, DoubleSupplier xAxis) {
    double DEADBAND = 0.5;

    return Commands.run(
        () -> {
          double speed = -yAxis.getAsDouble();
          double difference = xAxis.getAsDouble();

          if (Math.abs(speed) < 0.1) {
            speed = 0.0;
          }

          if (difference > DEADBAND) {
            climbSubsystem.setRightWinchSpeed(speed);
            climbSubsystem.setLeftWinchSpeed(0);
          } else if (difference < -DEADBAND) {
            climbSubsystem.setLeftWinchSpeed(speed);
            climbSubsystem.setRightWinchSpeed(0);
          } else {
            climbSubsystem.setRightWinchSpeed(speed);
            climbSubsystem.setLeftWinchSpeed(speed);
          }

          SmartDashboard.putNumber("speed climb", speed);

          // speed *= speed;
        },
        climbSubsystem);
  }

  public static Command raiseToLowChain(ClimbSubsystem climb) {
    return Commands.run(
        () -> {
          climb.setLeftWinchSpeed(1.0);
          climb.setRightWinchSpeed(1.0);
        },
        climb);
  }
}
