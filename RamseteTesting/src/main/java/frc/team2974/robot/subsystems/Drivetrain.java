package frc.team2974.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2974.robot.command.teleop.Drive;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;

import static frc.team2974.robot.RobotMap.*;

public class Drivetrain extends Subsystem {

    private AHRS ahrs;
    private DifferentialDriveOdometry driveOdometry;
    private DifferentialDriveKinematics differentialDriveKinematics;

    public static Drivetrain mDrivetrain = new Drivetrain();

    public Drivetrain() {
        motorLeft.setInverted(true);
        encoderLeft.setReverseDirection(true);

        try {
            ahrs = new AHRS(SPI.Port.kMXP);
            System.out.println("Initialized NavX on SPI bus.");
        } catch (RuntimeException e) {
            e.printStackTrace();
        }

        differentialDriveKinematics = new DifferentialDriveKinematics(0.07);

        driveOdometry = new DifferentialDriveOdometry(differentialDriveKinematics);
        compressor.stop();
    }

    public static Drivetrain getInstance() {
        return mDrivetrain;
    }

    public AHRS getAhrs() {
        return ahrs;
    }

    public Pose2d updateRobotPose() {
       return driveOdometry.update(encoderLeft.get(), encoderRight.get(), new Rotation2d(ahrs.getAngle()));
    }

    public Pose2d updateRobotPoseRelative(Pose2d relativePose) {
        return driveOdometry.update(encoderLeft.get(), encoderRight.get(), new Rotation2d(ahrs.getAngle())).relativeTo(relativePose);
    }

    public DifferentialDriveOdometry getDriveOdometry() {
        return driveOdometry;
    }

    public void resetRobotPose() {
        driveOdometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public int getRightEncoder() {
        return encoderRight.get();
    }

    public int getLeftEncoder() {
        return encoderLeft.get();
    }

    public void shiftDown() {
        pneumaticsShifter.set(true);
    }

    public void shiftUp() {
        pneumaticsShifter.set(false);
    }

    public boolean isShiftDown() {
        return pneumaticsShifter.get();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }

    public void resetEncoders() {
        encoderLeft.reset();
        encoderRight.reset();
    }

    public void setSpeeds(double leftThrottle, double rightThrottle) {
        motorLeft.set(leftThrottle);
        motorRight.set(rightThrottle);

    }
}
