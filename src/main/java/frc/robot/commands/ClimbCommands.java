package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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

  public static Command releaseTrapBar(ClimbSubsystem climb) {
    return Commands.run(
        () -> {
          climb.setTrapMotorPosition(220.939);
        },
        climb);
  }

  public static Command autoClimb(
      ClimbSubsystem climb, ArmSubsystem arm, ShooterSubsystem shooter) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
                Commands.run(
                    () -> {
                      if (1 - climb.m_WinchStringPotRight.get()
                          < Constants.ClimbConstants.WinchUpperRightLimit - 0.32665) {
                        climb.setRightWinchSpeed(1.0);
                      } else {
                        climb.setRightWinchSpeed(0.0);
                      }

                      if (climb.m_WinchMotorRight.get()
                          < Constants.ClimbConstants.WinchUpperLeftLimit - 0.32665) {
                        climb.setLeftWinchSpeed(1.0);
                      } else {
                        climb.setLeftWinchSpeed(0.0);
                      }
                    },
                    climb),
                Commands.run(
                        () -> {
                          shooter.SetIntake(-0.2f);
                          shooter.SetFeeder(-0.2f);
                        })
                    .withTimeout(2))
            .withTimeout(5),
        new SetArmPosition(arm, () -> ArmConstants.kArmPoseSource),
        Commands.run(
                () -> {
                  climb.setLeftWinchSpeed(1.0);
                  climb.setRightWinchSpeed(1.0);
                },
                climb)
            .withTimeout(5),
        Commands.run(
            () -> {
              shooter.SetIntake(-1);
              shooter.SetFeeder(-1);
            },
            climb));
  }
}
