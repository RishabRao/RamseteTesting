package lib.Utils;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import lib.Controller.RamseteController;
import lib.Controller.RamseteController.Outputs;

import lib.Kinematics.DifferentialDriveKinematics;

public class PFTrajectoryFollower extends Command {

  private RamseteController ramseteController;
  private DifferentialDriveKinematics differentialDriveKinematics;

  private int encoder_offset;
  private int encoder_tick_count;
  private double wheel_circumference;

  private Trajectory trajectory;
  private Segment seg;

  private Outputs ramseteOutput;

  private int segment;

  private double last_error;
  private double heading;


  public PFTrajectoryFollower(Trajectory trajectory) {
    this.trajectory = trajectory;
  }

  public PFTrajectoryFollower() {

  }

  public void setTrajectory(Trajectory traj) {
    this.trajectory = traj;
    reset();
  }

  /**
   * Configure the Encoders being used in the follower.
   * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
   *                              before you start to track
   * @param ticks_per_revolution  How many ticks per revolution the encoder has
   * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
   */

  public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
    encoder_offset = initial_position;
    encoder_tick_count = ticks_per_revolution;
    wheel_circumference = Math.PI * wheel_diameter;
  }

  /**
   * Reset the follower to start again. Encoders must be reconfigured.
   */
  public void reset() {
    last_error = 0; segment = 0;
  }

  public VelocityPair calculate() {

  if (segment < trajectory.length()) {
      Trajectory.Segment seg = trajectory.get(segment);
//      ramseteController.calculate(new Pose2d(seg.x, seg.y, new Rotation2d(seg.heading)),
//          1, 1, );
        //TODO: FIND ANGULAR AND LINEAR VELOCITY ^^^

      segment++;
      return new VelocityPair(0 , 0);
   }

  else return new VelocityPair(0, 0);
}


  /**
   * @return the desired heading of the current point in the trajectory
   */
  public double getHeading() {
    return heading;
  }

  /**
   * @return the current segment being operated on
   */
  public Trajectory.Segment getSegment() {
    return trajectory.get(segment);
  }

  /**
   * @return whether we have finished tracking this trajectory or not.
   */
  public boolean isFinished() {
    return segment >= trajectory.length();
  }
}
