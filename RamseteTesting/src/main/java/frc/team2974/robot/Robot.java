package frc.team2974.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.subsystems.Drivetrain;

import jaci.pathfinder.Trajectory;
import javax.management.*;
import java.lang.management.ManagementFactory;
import lib.Geometry.Rotation2d;

import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static OI oi;
    public static NetworkTable waltonDashboard;

    public Robot() {
        super(0.05);
    }

    @Override
    public void robotInit() {
            System.out.println("Initializing robot.");

            drivetrain = new Drivetrain();
            oi = new OI();
            drivetrain.resetEncoders();
            drivetrain.shiftDown();
    }

    private double getRIOCPUUse() throws MalformedObjectNameException, ReflectionException, InstanceNotFoundException {
        MBeanServer mbs = ManagementFactory.getPlatformMBeanServer();
        ObjectName name = ObjectName.getInstance("java.lang:type=OperatingSystem");
        AttributeList list = mbs.getAttributes(name, new String[]{"ProcessCpuLoad"});

        if (list.isEmpty()) return Double.NaN;

        Attribute att = (Attribute) list.get(0);
        Double value = (Double) att.getValue();

        // usually takes a couple of seconds before we get real values
        if (value == -1.0) return Double.NaN;
        // returns a percentage value with 1 decimal point precision
        return ((int) (value * 1000) / 10.0);
    }

    private double getRIORamUse() {
        long ramTotal = Runtime.getRuntime().totalMemory();
        long ramFree = Runtime.getRuntime().freeMemory();
        long ramUsed = ramTotal - ramFree;

        return ((ramUsed * 1.0) / ramTotal);
    }

    @Override
    public void robotPeriodic() {


    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {


    }

    @Override
    public void autonomousPeriodic() {




    }

    @Override
    public void teleopInit() {


        drivetrain.resetEncoders();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        SmartDashboard.putNumber("Left enc", encoderLeft.get());
        SmartDashboard.putNumber("Right enc", encoderRight.get());
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }
}
  