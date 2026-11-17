package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Sample Code that will not be implemented in the final robot code. 
 * This is just to show how to create a basic subsystem with a Spark motor controller.
 */
public class BasicSparkSubsystem {
    // Constants, normally stored in a separate Constants class, but for this example, we will keep them here.
    private final int sparkID = 0;
    private final boolean inverted = false;
    private final int currentLimit = 60;

    private final double slot0KP = 0.1;
    private final double slot0KI = 0.0;
    private final double slot0KD = 0.0;

    private final double slot0KS = 0.1;
    
    private final double slot1KP = 0.1;
    private final double slot1KI = 0.0;
    private final double slot1KD = 0.0;

    private final double slot1KS = 0.1;
    private final double slot1KV = 0.1;

    // Actual Motor Programming Objects
    private SparkMax motor;
    private SparkMaxConfig config;

    private SparkClosedLoopController controller;
    private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

    private boolean isBrakeMode = true;

    public BasicSparkSubsystem() {
        motor = new SparkMax(sparkID, MotorType.kBrushless);
        
        controller = motor.getClosedLoopController();

        // General Configuration
        config
            .inverted(inverted)// Motor and absolute inversions are opposite
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(currentLimit)
            .voltageCompensation(12);

        config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360/60);

        // Configuring PID and FF gains
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                slot0KP,
                slot0KI,
                slot0KD,
                ClosedLoopSlot.kSlot0
            )
            .pid(
                slot1KP,
                slot1KI,
                slot1KD,
                ClosedLoopSlot.kSlot1
            );
        
        config.closedLoop.feedForward
            // Slot 0 is for position control, so we only need kS
            .kS(slot0KS, ClosedLoopSlot.kSlot0)

            // Slot 1 is for velocity control, so we need both kS and kV
            .kS(slot1KS, ClosedLoopSlot.kSlot1)
            .kV(slot1KV, ClosedLoopSlot.kSlot1);

        // Configure signals
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
        
        // Apply configuration to the motor (persist safe parameters by default)
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runPercentOutput(double percentOutput) {
        motor.set(percentOutput);
    }

    public void runVoltage(double voltageVolts) {
        motor.setVoltage(voltageVolts);
    }

    public void runPosition(double position) {
        // Use slot 0 for position control
        setPIDSlot(ClosedLoopSlot.kSlot0);
        controller.setSetpoint(position, ControlType.kPosition, closedLoopSlot);
    }

    public void runVelocity(double velocity) {
        // Use slot 1 for velocity control
        setPIDSlot(ClosedLoopSlot.kSlot1);
        controller.setSetpoint(velocity, ControlType.kVelocity, closedLoopSlot);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setPIDSlot(ClosedLoopSlot slot) {
        closedLoopSlot = slot;
    }

    public void setIdleMode(boolean brake) {
        // Only change the idle mode if it is different from the current mode
        // Setting a new configuration takes cycle time on the robot, so we want to avoid doing it unnecessarily
        if(isBrakeMode != brake){
            SparkMaxConfig configuration = new SparkMaxConfig();
            configuration.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

            motor.configure(configuration, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            isBrakeMode = brake;
        }
    }

    public void reset() {
        motor.clearFaults();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
