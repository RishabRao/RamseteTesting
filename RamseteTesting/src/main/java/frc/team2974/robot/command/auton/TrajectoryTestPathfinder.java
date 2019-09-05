package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.command.teleop.Drive;
import frc.team2974.robot.subsystems.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;
import java.util.stream.Stream;

import lib.Geometry.Pose2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Utils.PathFollower;
import lib.Utils.VelocityPair;


public class TrajectoryTestPathfinder extends Command {

  private DifferentialDriveOdometry odometry;

  private Pose2d startingPose;
  private String trajectoryPath;

  private PathFollower pathFollower;

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

    double kBeta = 0.1;
    double kZeta = 2;

    this.pathFollower = new PathFollower(trajectory, 0.1, 0.5, 0.7);

  }


  @Override
  protected void execute() {

    Pose2d currentPose = Drivetrain.getInstance().updateRobotPose();
    VelocityPair velocityPair = pathFollower.getRobotVelocity(currentPose);



  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
