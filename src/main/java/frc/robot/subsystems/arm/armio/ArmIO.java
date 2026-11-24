package frc.robot.subsystems.arm.armio;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * ArmIO is an interface that defines the methods for using the physical electronics on the arm.
 */
public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public ArmIOData data = new ArmIOData(
            false,
            Degrees.of(0),
            DegreesPerSecond.of(0),
            Volts.of(0),
            Amps.of(0),
            Celsius.of(0)
        );
    }

    /**
     * ArmIOData is a record that holds the inputs from the arm subsystem.
     * Inputs are data given to us by the hardware on the arm that we need in our code.
     * 
     * @param connected whether the arm motor is connected with no faults/errors
     * @param position the position of the arm
     * @param velocity the velocity of the arm
     * @param appliedVoltage the voltage applied to the arm motor
     * @param current the current draw of the arm motor
     * @param tempCelsius the temperature of the arm motor in celsius
     * 
     */
    record ArmIOData(
        boolean connected,
        Angle position,
        AngularVelocity velocity,
        Voltage appliedVoltage,
        Current current,
        Temperature tempCelsius
    ) {}

    public void updateInputs(ArmIOInputs inputs);

    public void reset();

    public void runPercentOutput(double percentOutput);

    public void runVoltage(Voltage voltage);

    public void runPosition(Angle position, double arbFeedforward);

    public void stop();

    public void setPID(double kP, double kI, double kD, int slot);

    public void setKS(double kS, int slot);

    public void setPIDSlot(int slot);

    public void setBrakeMode(boolean brake);
}