package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command winchDrive(ClimbSubsystem climbSubsystem, DoubleSupplier yAxis) {
    double DEADBAND = 0.1f;

    return Commands.run(
        () -> {
          double speed = yAxis.getAsDouble();
          if (Math.abs(speed) <= DEADBAND) {
            speed = 0.0;
          }

          // speed *= speed;
          SmartDashboard.putNumber("Climb speed", speed);
          climbSubsystem.setWinchSpeed(MathUtil.clamp(speed, -1.0, 1.0));
        },
        climbSubsystem);
  }
}
