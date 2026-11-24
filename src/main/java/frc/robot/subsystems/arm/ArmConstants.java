package frc.robot.subsystems.arm;

public class ArmConstants {
    public static final int ARM_MOTOR_ID = 7;
    public static final double ARM_MOTOR_GEAR_RATIO = 2.0;
    public static final boolean ARM_MOTOR_INVERTED = true;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 35;
    public static final double ARM_MOTOR_VOLTAGE_COMPENSATION = 12.0;

    public static final double ARM_MOTOR_KP = 0.1;
    public static final double ARM_MOTOR_KI = 0.0;
    public static final double ARM_MOTOR_KD = 0.02;
    public static final double ARM_MOTOR_KS = 0.1;
    
    public static final double ARM_ABSOLUTE_ENCODER_ZERO_OFFSET = 0.1;
    public static final boolean ARM_ABSOLUTE_ENCODER_INVERTED = false;
    public static final double ARM_ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / ARM_MOTOR_GEAR_RATIO;
    public static final double ARM_ABSOLUTE_ENCODER_VELOCITY_CONVERSION_FACTOR = (360.0 / 60.0) / ARM_MOTOR_GEAR_RATIO;
}