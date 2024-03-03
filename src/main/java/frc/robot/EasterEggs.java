package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants.FieldLocations;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;

// We're so cooked.
public class EasterEggs {

  public static Command WaterWalkToLocation(Drive drive, FieldLocations location) {

    List<Translation2d> river = new ArrayList<Translation2d>();
    GoalEndState goal = new GoalEndState(0, new Rotation2d(0));
    switch (location) {
      case AMP:
        river =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(new Translation2d(1.8, 7.29), new Rotation2d(0.0)),
                Constants.FieldConstants.kAmpPose);
        goal = new GoalEndState(0, Rotation2d.fromDegrees(90.0));
        break;
      case SPEAKER:
        river =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(new Translation2d(1.9, 5.56), new Rotation2d(0.0)),
                Constants.FieldConstants.kSpeakerPose);
        goal = new GoalEndState(0, Rotation2d.fromDegrees(0.0));
        break;
      case SOURCE:
        river =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(new Translation2d(14.32, 1.07), new Rotation2d(0.0)),
                Constants.FieldConstants.kSourcePose);
        goal = new GoalEndState(0, Rotation2d.fromDegrees(-45.0));
        break;
      default:
        river =
            PathPlannerPath.bezierFromPoses(
                new Pose2d(new Translation2d(1.9, 5.56), new Rotation2d(0.0)),
                Constants.FieldConstants.kSpeakerPose);
        goal = new GoalEndState(0, Rotation2d.fromDegrees(0.0));
        break;
    }

    PathPlannerPath path =
        new PathPlannerPath(river, new PathConstraints(10, 10, 2 * Math.PI, 4 * Math.PI), goal);
    return AutoBuilder.followPath(path);
  }

  public static Command WaterWalkPath(Pose2d targetPose) {
    targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0, 0.0);
  }
}
