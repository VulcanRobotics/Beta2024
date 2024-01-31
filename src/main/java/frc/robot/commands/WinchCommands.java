package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class WinchCommands {

  private WinchCommands() {}

  public static Command winchDrive(ArmSubsystem winchSubsystem, DoubleSupplier yAxis) {
    double DEADBAND = 0.1f;

    return Commands.run(
        () -> {
          double speed = yAxis.getAsDouble();
          if (Math.abs(speed) <= DEADBAND) {
            speed = 0.0;
          }

          // speed *= speed;

          winchSubsystem.setWinchSpeed(MathUtil.clamp(speed, -1, 1));
        },
        winchSubsystem);
  }
}
