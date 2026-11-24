package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmConstants {
    public static final int CAN_ID = 7;
    public static final double GEAR_RATIO = 2.0;
    public static final boolean MOTOR_INVERTED = true;
    public static final Current CURRENT_LIMIT = Amps.of(35);
    public static final Voltage VOLTAGE_COMPENSATION = Volts.of(12.0);

    public static final double KP = 0.1;
    public static final double KI = 0.0;
    public static final double KD = 0.02;
    public static final double KS = 0.1;

    public static final Angle ABS_ENCODER_ZERO_OFFSET = Rotations.of(0.1);
    public static final boolean ABS_ENCODER_INVERTED = false;
    public static final double ABS_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / GEAR_RATIO;
    public static final double ABS_ENCODER_VELOCITY_CONVERSION_FACTOR = (360.0 / 60.0) / GEAR_RATIO;
}