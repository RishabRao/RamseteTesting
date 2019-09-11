package frc.team2974.robot.command.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.subsystems.Drivetrain;
import lib.Geometry.Pose2d;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Utils.TrajectoryFollower;
import lib.Utils.VelocityPair;
import lib.trajectory.Trajectory;

public class TrajectoryTestOTF extends Command {

    private DifferentialDriveOdometry odometry;
    private Trajectory trajectory;

    private Pose2d startingPose;

    private TrajectoryFollower trajectoryFollower;

    public TrajectoryTestOTF(Pose2d startingPose, Trajectory trajectory, double kBeta, double kZeta, double driveRadius, double dt) {
        requires(Drivetrain.getInstance());
        this.startingPose = startingPose;
        this.odometry = Drivetrain.getInstance().getDriveOdometry();
        this.trajectory = trajectory;
        trajectoryFollower = new TrajectoryFollower(trajectory, kBeta, kZeta, driveRadius, dt);
    }

    @Override
    protected void initialize() {



    }


    @Override
    protected void execute() {

        Pose2d currentPose = Drivetrain.getInstance().updateRobotPose();
        VelocityPair robotVelocities = trajectoryFollower.getRobotVelocity(currentPose);


    }

    @Override
    protected boolean isFinished() {
        return trajectoryFollower.isFinished();
    }
}

