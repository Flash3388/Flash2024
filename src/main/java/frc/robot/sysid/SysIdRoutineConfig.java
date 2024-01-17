package frc.robot.sysid;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SysIdRoutineConfig {

    /** The voltage ramp rate used for quasistatic test routines. */
    public final Measure<Velocity<Voltage>> rampRate;

    /** The step voltage output used for dynamic test routines. */
    public final Measure<Voltage> stepVoltage;

    /** Safety timeout for the test routine commands. */
    public final Measure<Time> timeout;

    /** Optional handle for recording test state in a third-party logging solution. */
    public final Consumer<SysIdRoutineLog.State> recordState;

    /**
     * Create a new configuration for a SysId test routine.
     *
     * @param rampRate The voltage ramp rate used for quasistatic test routines. Defaults to 1 volt
     *     per second if left null.
     * @param stepVoltage The step voltage output used for dynamic test routines. Defaults to 7
     *     volts if left null.
     * @param timeout Safety timeout for the test routine commands. Defaults to 10 seconds if left
     *     null.
     * @param recordState Optional handle for recording test state in a third-party logging
     *     solution. If provided, the test routine state will be passed to this callback instead of
     *     logged in WPILog.
     */
    public SysIdRoutineConfig(
            Measure<Velocity<Voltage>> rampRate,
            Measure<Voltage> stepVoltage,
            Measure<Time> timeout,
            Consumer<SysIdRoutineLog.State> recordState) {
        this.rampRate = rampRate != null ? rampRate : Volts.of(1).per(Seconds.of(1));
        this.stepVoltage = stepVoltage != null ? stepVoltage : Volts.of(7);
        this.timeout = timeout != null ? timeout : Seconds.of(10);
        this.recordState = recordState;
    }

    /**
     * Create a new configuration for a SysId test routine.
     *
     * @param rampRate The voltage ramp rate used for quasistatic test routines. Defaults to 1 volt
     *     per second if left null.
     * @param stepVoltage The step voltage output used for dynamic test routines. Defaults to 7
     *     volts if left null.
     * @param timeout Safety timeout for the test routine commands. Defaults to 10 seconds if left
     *     null.
     */
    public SysIdRoutineConfig(
            Measure<Velocity<Voltage>> rampRate, Measure<Voltage> stepVoltage, Measure<Time> timeout) {
        this(rampRate, stepVoltage, timeout, null);
    }

    /**
     * Create a default configuration for a SysId test routine with all default settings.
     *
     * <p>rampRate: 1 volt/sec
     *
     * <p>stepVoltage: 7 volts
     *
     * <p>timeout: 10 seconds
     */
    public SysIdRoutineConfig() {
        this(null, null, null, null);
    }
}
