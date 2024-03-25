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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants.FieldLocations;
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
  private final PhotonVisionSubsystem vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController buttonBoard = new CommandXboxController(2);

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
        vision = new PhotonVisionSubsystem(drive);
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
        vision = new PhotonVisionSubsystem(drive);
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
        vision = new PhotonVisionSubsystem(drive);
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

    // NamedCommands.registerCommand("ToggleShoot", new ShootToggle(shooterSubsystem).asProxy());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    /* autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */

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

    driverController
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                DriveCommands.driveWhileAiming(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    drive::calculateShootingDirectPose),
                Commands.run(
                    () -> armSubsystem.setArmPosition(drive::getArmShootingAngle), armSubsystem)));

    // driverController
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> drive.setPose(new Pose2d(new Translation2d(0.0, 0.0), new
    // Rotation2d(0.0))),
    //             drive));

    driverController
        .leftBumper()
        .whileTrue(EasterEggs.WaterWalkToLocation(drive, FieldLocations.AMP));

    driverController
        .rightBumper()
        .whileTrue(ShooterTargeting.shuttleShoot(drive, shooterSubsystem, armSubsystem));

    driverController
        .rightTrigger()
        .whileTrue(
            new ParallelDeadlineGroup(
                new IntakeCommand(shooterSubsystem),
                DriveCommands.driveWhileAiming(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    drive::calculateIntakePose),
                new SetArmPosition(armSubsystem, () -> ArmConstants.kArmPoseIntake)));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.isUsingVision = !drive.isUsingVision, drive)
                .ignoringDisable(true));

    driverController.povUp().onTrue(new InstantCommand(() -> ArmConstants.kVariable += 0.1));
    driverController.povDown().onTrue(new InstantCommand(() -> ArmConstants.kVariable -= 0.1));

    // Operator

    armSubsystem.setDefaultCommand(
        WinchCommands.winchDrive(armSubsystem, () -> operatorController.getLeftY()));

    climbSubsystem.setDefaultCommand(
        ClimbCommands.winchDrive(
            climbSubsystem,
            () -> operatorController.getRightY(),
            () -> operatorController.getRightX()));
    // new InstantCommand(() -> climbSubsystem.setWinchSpeed(operatorController.getRightX())));
    // ClimbCommands.winchDrive(climbSubsystem, () -> operatorController.getYaw());

    /*operatorController.povRight().onTrue(new InstantCommand(() -> climbSubsystem.winchDown()));
    operatorController.povLeft().onTrue(new InstantCommand(() -> climbSubsystem.winchUp()));*/

    // Button Board

    buttonBoard
        .a()
        .whileTrue(
            new InstantCommand(
                () -> {
                  shooterSubsystem.SetIntakeMotor("Upper", shooterSubsystem.savedIntakeUpperSpeed);
                  shooterSubsystem.SetFeeder(-0.5f);
                }));
    buttonBoard
        .b()
        .whileTrue(
            new InstantCommand(
                () -> {
                  shooterSubsystem.SetIntakeMotor("Lower", shooterSubsystem.savedIntakeLowerSpeed);
                  shooterSubsystem.SetFeeder(-0.5f);
                }));

    buttonBoard
        .a()
        .onFalse(
            new InstantCommand(
                () -> {
                  shooterSubsystem.SetIntakeMotor("Upper", 0);
                  shooterSubsystem.SetFeeder(0);
                }));
    buttonBoard
        .b()
        .onFalse(
            new InstantCommand(
                () -> {
                  shooterSubsystem.SetIntakeMotor("Lower", 0);
                  shooterSubsystem.SetFeeder(0);
                }));

    buttonBoard
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> MathUtil.clamp(shooterSubsystem.savedIntakeUpperSpeed += 0.1, -1, 0)));
    buttonBoard
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> MathUtil.clamp(shooterSubsystem.savedIntakeUpperSpeed -= 0.1, -1, 0)));

    buttonBoard
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> MathUtil.clamp(shooterSubsystem.savedIntakeLowerSpeed += 0.1, -1, 0)));
    buttonBoard
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> MathUtil.clamp(shooterSubsystem.savedIntakeLowerSpeed -= 0.1, -1, 0)));

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
    operatorController
        .b()
        .whileTrue(new SetArmPosition(armSubsystem, () -> ArmConstants.kArmPosePodium));

    operatorController
        .rightStick()
        .onTrue(Commands.runOnce(() -> shooterSubsystem.SetIntake(-0.02f), shooterSubsystem));

    operatorController
        .button(8)
        .whileTrue(
            new ParallelCommandGroup(
                new SetArmPosition(armSubsystem, () -> Constants.ArmConstants.kArmPoseAmp),
                ClimbCommands.raiseToLowChain(climbSubsystem)));

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
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  //                  shooterSubsystem.savedShootSpeed =
                  //                      Math.max(shooterSubsystem.savedShootSpeed -= 0.1, 0);
                  drive.armAngleOffset += 1.0;
                }));

    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  //                  shooterSubsystem.savedShootSpeed =
                  //                      Math.min(shooterSubsystem.savedShootSpeed += 0.1, 1);
                  drive.armAngleOffset -= 1.0;
                }));
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
