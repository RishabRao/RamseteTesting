package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.command.teleop.Drive;
import frc.team2974.robot.subsystems.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

import lib.Geometry.Pose2d;
import lib.Utils.PathFollower;


public class TrajectoryTest extends Command {

  private Notifier _notifier;
  private Runnable runnable;
  private Drivetrain drivetrain;

  private Pose2d startingPose;

  public TrajectoryTest(Pose2d startPose) {
    startingPose = startPose;
  }

  @Override
  protected void initialize() {

    String trajectoryPath = "fix this";


    Trajectory trajectory = null;

    try {
      trajectory = PathfinderFRC.getTrajectory(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    double kBeta = 0.1;
    double kZeta = 2;

    PathFollower pathFollower = new PathFollower(trajectory, 0.1, 0.5, 0.7);
    Notifier notifier = new Notifier(() -> pathFollower.getRobotVelocity(Drivetrain.getInstance().updateRobotPoseRelative(startingPose)));
    _notifier.startPeriodic(0.02);
  }


  @Override
  protected void execute() {

  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
