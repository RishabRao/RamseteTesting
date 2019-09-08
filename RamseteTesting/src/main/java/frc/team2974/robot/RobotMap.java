package frc.team2974.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;

import static frc.team2974.robot.Config.Hardware.*;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides
 * flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public final class RobotMap {

    public static final Talon motorLeft;
    public static final Talon motorRight;

    public static final Encoder encoderLeft;
    public static final Encoder encoderRight;

    public static final Compressor compressor;
    public static final Solenoid pneumaticsShifter;

    public static final CANSparkMax rightWheelsMaster = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax rightWheelsSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static final CANSparkMax leftWheelsMaster = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax leftWheelsSlave = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    static {

        motorLeft = new Talon(LEFT_MOTOR_CHANNEL);
        motorRight = new Talon(RIGHT_MOTOR_CHANNEL);

        encoderRight = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1),
                new DigitalInput(RIGHT_ENCODER_CHANNEL2));
        encoderLeft = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1),
                new DigitalInput(LEFT_ENCODER_CHANNEL2));

        compressor = new Compressor();
        pneumaticsShifter = new Solenoid(SHIFTER_CHANNEL);
    }

    private RobotMap() {
    }
}
