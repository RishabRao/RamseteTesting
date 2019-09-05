package frc.team2974.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.subsystems.Drivetrain;

import static frc.team2974.robot.OI.*;

public class Drive extends Command {

    public Drive() {
        requires(Drivetrain.getInstance());
    }

    public double getLeftThrottle() {
        if (Math.abs(leftJoystick.getY()) < 0.1) {
            return 0;
        }
        return leftJoystick.getY();
    }

    public double getRightThrottle() {
        if (Math.abs(rightJoystick.getY()) < 0.1) {
            return 0;
        }
        return rightJoystick.getY();
    }

    private void tankDrive() {
        Drivetrain.getInstance().setSpeeds(getLeftThrottle(), getRightThrottle());
    }

    @Override
    protected void initialize() {
        System.out.println("Initialized drive.");
    }

    @Override
    protected void execute() {
        tankDrive();
        Drivetrain.getInstance().updateRobotPose();

        if (shiftUp.get()) {
            Drivetrain.getInstance().shiftUp();
        }

        if (shiftDown.get()) {
            Drivetrain.getInstance().shiftDown();
        }
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        System.out.println("Ended drive.");

        Drivetrain.getInstance().setSpeeds(0, 0);
    }

    protected void interrupted() {
        end();
    }
}
