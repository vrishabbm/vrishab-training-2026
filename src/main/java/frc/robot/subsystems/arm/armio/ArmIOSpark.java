package frc.robot.subsystems.arm.armio;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmIOSpark implements ArmIO {
    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkMaxConfig config;

    private SparkClosedLoopController controller;
    private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

    boolean isBrakeMode = true;

    public ArmIOSpark() {
        motor = new SparkMax(ArmConstants.CAN_ID, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        
        config = new SparkMaxConfig();

        controller = motor.getClosedLoopController();

        // Configuration
        config.inverted(ArmConstants.MOTOR_INVERTED);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit((int) ArmConstants.CURRENT_LIMIT.in(Amps));
        config.voltageCompensation(ArmConstants.VOLTAGE_COMPENSATION.in(Volts));

        config.absoluteEncoder
            .inverted(ArmConstants.ABS_ENCODER_INVERTED)
            .zeroOffset(ArmConstants.ABS_ENCODER_ZERO_OFFSET.in(Rotations))
            .positionConversionFactor(ArmConstants.ABS_ENCODER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ArmConstants.ABS_ENCODER_VELOCITY_CONVERSION_FACTOR);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(ArmConstants.KP)
            .i(ArmConstants.KI)
            .d(ArmConstants.KD);
        
        config.signals
            .appliedOutputPeriodMs(20) // See what voltage motor is using
            .outputCurrentPeriodMs(100) // Catch current spikes
            .busVoltagePeriodMs(100) // Monitor battery voltage
            .faultsAlwaysOn(true)
            .faultsPeriodMs(1000) // Updates once a second, faults don't need high update rate
            .iAccumulationAlwaysOn(true) // tracks the integral accumulator for PID
            .motorTemperaturePeriodMs(1000)
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20) // Updates at robot loop rate
            .warningsAlwaysOn(true)
            .warningsPeriodMs(1000); // Updates once a second, warnings don't need high update rate

        motor.clearFaults();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /****************************************** ARM IO IMPLEMENTED METHODS ******************************************/
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.data = new ArmIOData(
            motor.getLastError() == REVLibError.kOk,
            Degrees.of(encoder.getPosition()),
            DegreesPerSecond.of(encoder.getVelocity()),
            Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()),
            Amps.of(motor.getOutputCurrent()),
            Celsius.of(motor.getMotorTemperature())
        );
    }

    @Override
    public void reset() {

    }

    @Override
    public void runPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    @Override
    public void runVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void runPosition(Angle position, double arbFeedforward) {
        controller.setSetpoint(position.in(Degrees), ControlType.kPosition, closedLoopSlot, arbFeedforward);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double kP, double kI, double kD, int slot) {
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.fromInt(slot);

        SparkMaxConfig tempConfig = new SparkMaxConfig();
        tempConfig.closedLoop.p(kP, closedLoopSlot);
        tempConfig.closedLoop.i(kI, closedLoopSlot);
        tempConfig.closedLoop.d(kD, closedLoopSlot);

        motor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setKS(double kS, int slot) {
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.fromInt(slot);
        
        SparkMaxConfig tempConfig = new SparkMaxConfig();
        tempConfig.closedLoop.feedForward.kS(kS, closedLoopSlot);

        motor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPIDSlot(int slot) {
        closedLoopSlot = ClosedLoopSlot.fromInt(slot);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        if(isBrakeMode != brake){
            SparkMaxConfig tempConfig = new SparkMaxConfig();
            tempConfig.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

            motor.configure(tempConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            isBrakeMode = brake;
        }
    }
}
