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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
// import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
// import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
// import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final PhotonVisionSubsystem vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(false),
        //         new ModuleIOSparkMax(0),
        //         new ModuleIOSparkMax(1),
        //         new ModuleIOSparkMax(2),
        //         new ModuleIOSparkMax(3));
        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        drive =
            new Drive(
                // new GyroIOPigeon2(true),
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        flywheel = new Flywheel(new FlywheelIOSim());
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
        flywheel = new Flywheel(new FlywheelIOSim());
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
        flywheel = new Flywheel(new FlywheelIO() {});
        shooterSubsystem = new ShooterSubsystem();
        armSubsystem = new ArmSubsystem();
        climbSubsystem = new ClimbSubsystem();
        vision = new PhotonVisionSubsystem(drive);
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));

    NamedCommands.registerCommand(
        "Intake", new IntakeCommand(shooterSubsystem, false).withTimeout(3));

    NamedCommands.registerCommand(
        "ArmToIntake", new SetArmPosition(armSubsystem, 0).withTimeout(2));

    NamedCommands.registerCommand("Rev", new RevCommand(shooterSubsystem, true).withTimeout(3));
    NamedCommands.registerCommand("Shoot", new ShootCommand(shooterSubsystem).withTimeout(3));
    NamedCommands.registerCommand(
        "RevShoot",
        new ParallelCommandGroup(
                new RevCommand(shooterSubsystem, false), new ShootCommand(shooterSubsystem))
            .withTimeout(3));

    // NamedCommands.registerCommand("ToggleShoot", new ShootToggle(shooterSubsystem).asProxy());

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
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
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            new DriveToPosition(drive, new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0))));
    // Operator

    armSubsystem.setDefaultCommand(
        WinchCommands.winchDrive(armSubsystem, () -> operatorController.getLeftY()));

    climbSubsystem.setDefaultCommand(
        ClimbCommands.winchDrive(
            climbSubsystem,
            () -> operatorController.getRawAxis(3),
            operatorController.povUp(),
            operatorController.povDown()));
    // new InstantCommand(() -> climbSubsystem.setWinchSpeed(operatorController.getRightX())));
    // ClimbCommands.winchDrive(climbSubsystem, () -> operatorController.getYaw());

    /*operatorController.povRight().onTrue(new InstantCommand(() -> climbSubsystem.winchDown()));
    operatorController.povLeft().onTrue(new InstantCommand(() -> climbSubsystem.winchUp()));*/

    // Manual Arm Controls
    operatorController
        .button(1)
        .whileTrue(new SetArmPosition(armSubsystem, ArmConstants.kArmPoseIntake));
    operatorController
        .button(2)
        .whileTrue(new SetArmPosition(armSubsystem, ArmConstants.kArmPoseTrap));

    // Reset arm guide motor encoder to 0 rotations
    operatorController
        .button(10)
        .onTrue(
            new InstantCommand(
                () -> {
                  armSubsystem.calibrateTalonEncoder();
                }));

    operatorController.button(3).onTrue(new LockWinchCommand(climbSubsystem));

    // Shooter and intake commands
    operatorController.button(8).whileTrue(new RevCommand(shooterSubsystem, false));
    operatorController.button(6).whileTrue(new ShootCommand(shooterSubsystem));
    // operatorController.button(6).onTrue(new ShootToggle(shooterSubsystem));
    operatorController.button(7).whileTrue(new IntakeCommand(shooterSubsystem, false));
    operatorController.button(5).whileTrue(new DispenseCommand(shooterSubsystem));
    operatorController
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  // shooterSubsystem.savedShootSpeed -= 0.1;
                  shooterSubsystem.savedShootSpeed =
                      Math.max(shooterSubsystem.savedShootSpeed -= 0.1, 0);
                }));

    operatorController
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  // shooterSubsystem.savedShootSpeed += 0.1;
                  shooterSubsystem.savedShootSpeed =
                      Math.min(shooterSubsystem.savedShootSpeed += 0.1, 1);
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
