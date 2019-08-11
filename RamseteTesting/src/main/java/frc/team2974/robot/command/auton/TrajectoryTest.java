package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Watchable;
import lib.Controller.RamseteController;


public class TrajectoryTest extends Command {

  private Notifier _notifier;
  private Runnable runnable;

  @Override
  protected void initialize() {

    String leftTrajectoryFilepath = "fix this";
    String rightTrajectoryFilepath = "fix this";

    Trajectory leftTrajectory = null;
    Trajectory rightTrajectory = null;

    try {
      leftTrajectory = PathfinderFRC.getTrajectory(leftTrajectoryFilepath);
      rightTrajectory = PathfinderFRC.getTrajectory(rightTrajectoryFilepath);
    } catch (IOException e) {
      e.printStackTrace();
    }
    runnable = new Runnable() {
      @Override
      public void run() {
      }
    };
    _notifier = new Notifier(runnable);

    _notifier.startPeriodic(0.05);

    double kBeta = 0.1;
    double kZeta = 2;


    RamseteController ramseteController = new RamseteController(kBeta, kZeta);



  }


  @Override
  protected void execute() {

  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
