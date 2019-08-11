package frc.team2974.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2974.robot.command.teleop.Drive;
import static frc.team2974.robot.RobotMap.*;

public class Drivetrain extends Subsystem {

    private AHRS ahrs;

    public Drivetrain() {
        motorLeft.setInverted(true);
        encoderLeft.setReverseDirection(true);

        try {
            ahrs = new AHRS(SPI.Port.kMXP);
            System.out.println("Initialized NavX on SPI bus.");
        } catch (RuntimeException e) {
            e.printStackTrace();
        }

        compressor.stop();
    }

    public AHRS getAhrs() {
        return ahrs;
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
