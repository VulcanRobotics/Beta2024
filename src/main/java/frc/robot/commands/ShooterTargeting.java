package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class ShooterTargeting extends Command{
    ShooterSubsystem shooter;
    ArmSubsystem arm;
    Drive drive;

    public ShooterTargeting(Drive drive, ArmSubsystem arm, ShooterSubsystem shooter) {
        addRequirements(arm, drive, shooter);
        this.arm = arm;
        this.drive = drive;
        this.shooter = shooter;
    }

    
    
}
