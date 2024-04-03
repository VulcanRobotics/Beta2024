// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
// import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
// import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
// import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Flywheel flywheel;
  public ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimbSubsystem climbSubsystem;
  // private final PhotonVisionSubsystem vision;
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                // new GyroIOPigeon2(true),
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));

        // flywheel = new Flywheel(new FlywheelIOSim());
        shooterSubsystem = new ShooterSubsystem();
        armSubsystem = new ArmSubsystem();
        climbSubsystem = new ClimbSubsystem();
        // vision = new PhotonVisionSubsystem(drive);
        vision = new VisionSubsystem(drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // flywheel = new Flywheel(new FlywheelIOSim());
        shooterSubsystem = new ShooterSubsystem();
        armSubsystem = new ArmSubsystem();
        climbSubsystem = new ClimbSubsystem();
        // vision = new PhotonVisionSubsystem(drive);
        vision = new VisionSubsystem(drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        shooterSubsystem = new ShooterSubsystem();
        armSubsystem = new ArmSubsystem();
        climbSubsystem = new ClimbSubsystem();
        // vision = new PhotonVisionSubsystem(drive);
        vision = new VisionSubsystem(drive);
        break;
    }

    // Set up auto routines

    NamedCommands.registerCommand("Intake", new IntakeCommand(shooterSubsystem));
    NamedCommands.registerCommand("IntakeNoDeadline", new IntakeCommand(shooterSubsystem));

    NamedCommands.registerCommand(
        "RampShooter",
        Commands.runOnce(
            () ->
                shooterSubsystem.setShooterVelocity(
                    Constants.ShooterConstants.kShooterTargetVelocity),
            shooterSubsystem));

    NamedCommands.registerCommand(
        "KillShooter",
        Commands.runOnce(() -> shooterSubsystem.setShooterVelocity(0.0), shooterSubsystem));

    NamedCommands.registerCommand(
        "ArmToIntake", new SetArmPosition(armSubsystem, () -> 0).withTimeout(2));

    NamedCommands.registerCommand(
        "ArmToAmp", new SetArmPosition(armSubsystem, () -> 90).withTimeout(2));

    NamedCommands.registerCommand(
        "AutoTargetShoot",
        ShooterTargeting.shootAtTarget(drive, shooterSubsystem, armSubsystem).withTimeout(3.0));

    NamedCommands.registerCommand(
        "NoteTrackMode",
        Commands.runOnce(
            () -> {
              PPHolonomicDriveController.setRotationTargetOverride(drive::calculateIntakeAngle);
            }));

    NamedCommands.registerCommand(
        "Rev", new RevCommand(shooterSubsystem, armSubsystem, drive).withTimeout(3));

    NamedCommands.registerCommand(
        "Shoot", new ShootCommand(shooterSubsystem, armSubsystem, drive).withTimeout(3));

    NamedCommands.registerCommand(
        "RevShoot",
        new SequentialCommandGroup(
                new IntakeCommand(shooterSubsystem),
                new ParallelDeadlineGroup(
                    new ShootCommand(shooterSubsystem, armSubsystem, drive),
                    new RevCommand(shooterSubsystem, armSubsystem, drive)))
            .withTimeout(3.0));

    NamedCommands.registerCommand(
        "OverrideRotationAim",
        Commands.runOnce(
            () ->
                PPHolonomicDriveController.setRotationTargetOverride(
                    drive::calculateShootingAngle)));

    NamedCommands.registerCommand(
        "UsePathRotation",
        Commands.runOnce(
            () -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty())));

    NamedCommands.registerCommand(
        "AimArmShoot", ShooterTargeting.aimArmAndShoot(drive, shooterSubsystem, armSubsystem));

    // NamedCommands.registerCommand("ToggleShoot", new ShootToggle(shooterSubsystem).asProxy());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger rumbleTrigger = new Trigger(shooterSubsystem::hasNote);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

                      drive.zeroGyro();
                    },
                    drive)
                .ignoringDisable(true));

    driverController
        .a()
        .whileTrue(
            new ParallelCommandGroup(
                DriveCommands.driveWhileAiming(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    drive::calculateShootingPose),
                Commands.run(
                    () -> armSubsystem.setArmPosition(drive::getArmShootingAngle), armSubsystem)));

    // driverController
    //     .b()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             DriveCommands.driveWhileAiming(
    //                 drive,
    //                 () -> -driverController.getLeftY(),
    //                 () -> -driverController.getLeftX(),
    //                 drive::calculateShootingDirectPose),
    //             Commands.run(
    //                 () -> armSubsystem.setArmPosition(drive::getArmShootingAngle),
    // armSubsystem)));

    driverController
        .rightBumper()
        .whileTrue(
            new ParallelDeadlineGroup(
                new IntakeCommand(shooterSubsystem),
                DriveCommands.driveWhileAiming(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        drive::getShuttlePoseConstant)
                    .alongWith(new SetArmPosition(armSubsystem, () -> 10.0))));

    driverController
        .rightTrigger()
        .whileTrue(
            DriveCommands.driveWhileNoteTracking(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.isUsingVision = !drive.isUsingVision, drive)
                .ignoringDisable(true));

    driverController.b().whileTrue(ClimbCommands.releaseTrapBar(climbSubsystem));

    driverController.leftBumper().whileTrue(DriveCommands.driveToAmp(drive, drive::getPose));

    driverController.povUp().onTrue(new InstantCommand(() -> ArmConstants.kVariable += 0.1));
    driverController.povDown().onTrue(new InstantCommand(() -> ArmConstants.kVariable -= 0.1));

    // Both controllers will rumble for a second when they intake a note.
    rumbleTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                })
            .andThen(new WaitCommand(1.0))
            .finallyDo(
                () -> {
                  driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }));

    rumbleTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                })
            .andThen(new WaitCommand(1.0))
            .finallyDo(
                () -> {
                  operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                }));

    // Operator

    armSubsystem.setDefaultCommand(
        WinchCommands.winchDrive(armSubsystem, () -> operatorController.getLeftY()));

    climbSubsystem.setDefaultCommand(
        ClimbCommands.winchDrive(
            climbSubsystem,
            () -> operatorController.getRightY(),
            () -> operatorController.getRightX()));

    // Manual Arm Controls
    operatorController
        .x()
        .whileTrue(new SetArmPosition(armSubsystem, () -> ArmConstants.kArmPoseIntake));
    operatorController
        .a()
        .whileTrue(new SetArmPosition(armSubsystem, () -> ArmConstants.kArmPoseTrap));
    operatorController
        .y()
        .whileTrue(new SetArmPosition(armSubsystem, () -> ArmConstants.kArmPoseSource));

    operatorController.b().whileTrue(ClimbCommands.releaseTrapAndRaiseToLowChain(climbSubsystem));
    operatorController.b().onFalse(ClimbCommands.stopTrapBar(climbSubsystem));

    // operatorController
    //     .povUp()
    //     .whileTrue(new InstantCommand(() -> climbSubsystem.m_TrapMotor.set(0.25f)));
    // operatorController.povUp().onFalse(new InstantCommand(() ->
    // climbSubsystem.m_TrapMotor.set(0)));

    operatorController
        .povDown()
        .whileTrue(new InstantCommand(() -> climbSubsystem.m_TrapMotor.set(-0.25f)));
    operatorController
        .povDown()
        .onFalse(new InstantCommand(() -> climbSubsystem.m_TrapMotor.set(0)));

    operatorController
        .rightStick()
        .onTrue(Commands.runOnce(() -> shooterSubsystem.SetIntake(-0.02f), shooterSubsystem));

    // Shooter and intake commands
    operatorController
        .rightTrigger()
        .whileTrue(new RevCommand(shooterSubsystem, armSubsystem, drive));

    operatorController
        .rightBumper()
        .whileTrue(new ShootCommand(shooterSubsystem, armSubsystem, drive));
    operatorController.leftTrigger().whileTrue(new IntakeCommand(shooterSubsystem));
    operatorController.leftBumper().whileTrue(new DispenseCommand(shooterSubsystem));
    operatorController
        .button(7)
        .onTrue(
            new InstantCommand(
                () -> {
                  //                  shooterSubsystem.savedShootSpeed =
                  //                      Math.max(shooterSubsystem.savedShootSpeed -= 0.1, 0);
                  drive.armAngleOffset += 1.0;
                }));

    operatorController
        .button(8)
        .onTrue(
            new InstantCommand(
                () -> {
                  //                  shooterSubsystem.savedShootSpeed =
                  //                      Math.min(shooterSubsystem.savedShootSpeed += 0.1, 1);
                  drive.armAngleOffset -= 1.0;
                }));

    // revTrigger.whileTrue(new RevCommand(shooterSubsystem, armSubsystem, drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
