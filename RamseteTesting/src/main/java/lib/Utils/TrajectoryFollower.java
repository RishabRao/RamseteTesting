package lib.Utils;

import edu.wpi.first.wpilibj.Timer;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.trajectory.Trajectory;
import lib.trajectory.Trajectory.State;

public class TrajectoryFollower {

    private Trajectory trajectory;
    private double currentTime;

    private double drive_radius;

    private double dt;

    private RamseteController ramseteController;

    public TrajectoryFollower(Trajectory trajectory, double kBeta, double kZeta, double driveRadius, double dt) {
        this.ramseteController = new RamseteController(kBeta, kZeta);
        this.trajectory = trajectory;
        this.drive_radius = driveRadius;
        this.currentTime = 0;
        this.dt = dt;
    }

    /**
     * @param pose : The current pose of the robot
     * @return A velocity pair of the left and right wheel speeds
     */
    public VelocityPair getRobotVelocity(Pose2d pose) {

        if (isFinished())
            return new VelocityPair(0, 0);

        State currentState = trajectory.sample(currentTime);
        State nextState = trajectory.sample(currentTime + dt);

        double trajX = currentState.poseMeters.getTranslation().getX();
        double trajY = currentState.poseMeters.getTranslation().getY();

        // Calculate X and Y error
        double xError = trajX - pose.getTranslation().getX();
        double yError = trajY - pose.getTranslation().getY();

        // Calculate Linear Velocity

        double sv = currentState.velocityMetersPerSecond;

        // Calculate Angular Velocity

        double sw = isFinished() ? 0 : (nextState.poseMeters.getRotation().getDegrees() - currentState.poseMeters.getRotation().getDegrees())/dt;

        // Calculate linear and angular velocity based on errors

        RamseteController.Outputs ramseteOutputs = ramseteController
                .calculate(new Pose2d(currentState.poseMeters.getTranslation().getX(), currentState.poseMeters.getTranslation().getY(), new Rotation2d(currentState.poseMeters.getRotation().getDegrees())),
                        sv, sw, pose);

        ChassisSpeeds speeds = new ChassisSpeeds(ramseteOutputs.linearVelocity,0,ramseteOutputs.angularVelocity);

        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(drive_radius);

        currentTime = currentTime + dt;

        return new VelocityPair(kinematics.toWheelSpeeds(speeds).left, kinematics.toWheelSpeeds(speeds).right);
    }

    public boolean isFinished() {
        return currentTime >= trajectory.getTotalTimeSeconds();
    }
}