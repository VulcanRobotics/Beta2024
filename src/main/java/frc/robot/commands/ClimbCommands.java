package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.function.DoubleSupplier;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command winchDrive(
      ClimbSubsystem climbSubsystem, DoubleSupplier yAxis, Trigger dpadUp, Trigger dpadDown) {
    double DEADBAND = 0.1f;

    return Commands.run(
        () -> {
          double rightSpeed = yAxis.getAsDouble();
          double leftSpeed = 0;

          if (Math.abs(rightSpeed) <= DEADBAND) {
            rightSpeed = 0.0;
          }

          if (dpadUp.getAsBoolean()) {
            leftSpeed = 1;
          } else if (dpadDown.getAsBoolean()) {
            leftSpeed = -1;
          } else {
            leftSpeed = 0;
          }

          // speed *= speed;
          climbSubsystem.setRightWinchSpeed(MathUtil.clamp(-rightSpeed, -1.0, 1.0));
          climbSubsystem.setLeftWinchSpeed(leftSpeed);
        },
        climbSubsystem);
  }

  public static Command raiseToLowChain(ClimbSubsystem climb) {
    return Commands.run(
        () -> {
          if (climb.m_WinchMotorRight.getPosition().getValueAsDouble()
              > (climb.rightLimitMotor - Constants.ClimbConstants.kRightTopDistanceFromChain)) {
            climb.setRightWinchSpeed(1.0);
          } else {
            climb.setRightWinchSpeed(0.0);
          }

          if (climb.m_WinchMotorLeft.getPosition().getValueAsDouble()
              < (climb.leftLimitMotor + Constants.ClimbConstants.kLeftTopDistanceFromChain)) {
            climb.setLeftWinchSpeed(1.0);
          } else {
            climb.setLeftWinchSpeed(0.0);
          }
        },
        climb);
  }
}
