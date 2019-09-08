package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.subsystems.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

import lib.Geometry.Pose2d;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Utils.PathFinderFollower;
import lib.Utils.VelocityPair;


public class TrajectoryTestPathfinder extends Command {

  private DifferentialDriveOdometry odometry;

  private Pose2d startingPose;
  private String trajectoryPath;

  private PathFinderFollower pathFinderFollower;

  public TrajectoryTestPathfinder(Pose2d startingPose, String trajectoryPath) {
    requires(Drivetrain.getInstance());
    this.startingPose = startingPose;
    this.trajectoryPath = trajectoryPath;
    this.odometry = Drivetrain.getInstance().getDriveOdometry();
  }

  @Override
  protected void initialize() {

    Trajectory trajectory = null;

    try {
      trajectory = PathfinderFRC.getTrajectory(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    double kBeta = 2;
    double kZeta = 0.7;

    this.pathFinderFollower = new PathFinderFollower(trajectory, kBeta, kZeta, 0.7);

  }


  @Override
  protected void execute() {

    Pose2d currentPose = Drivetrain.getInstance().updateRobotPose();
    VelocityPair velocityPair = pathFinderFollower.getRobotVelocity(currentPose);

  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
