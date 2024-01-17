package frc.robot.sysid;

import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.util.function.Consumer;

public class SysIdRoutineMechanism {

    /** Sends the SysId-specified drive signal to the mechanism motors during test routines. */
    public final Consumer<Measure<Voltage>> drive;

    /**
     * Returns measured data (voltages, positions, velocities) of the mechanism motors during test
     * routines.
     */
    public final Consumer<SysIdRoutineLog> log;

    /** The subsystem containing the motor(s) that is (or are) being characterized. */
    public final Subsystem subsystem;

    /** The name of the mechanism being tested. */
    public final String name;

    /**
     * Create a new mechanism specification for a SysId routine.
     *
     * @param drive Sends the SysId-specified drive signal to the mechanism motors during test
     *     routines.
     * @param log Returns measured data of the mechanism motors during test routines. To return
     *     data, call `motor(string motorName)` on the supplied `SysIdRoutineLog` instance, and then
     *     call one or more of the chainable logging handles (e.g. `voltage`) on the returned
     *     `MotorLog`. Multiple motors can be logged in a single callback by calling `motor`
     *     multiple times.
     * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
     *     Will be declared as a requirement for the returned test commands.
     * @param name The name of the mechanism being tested. Will be appended to the log entry title
     *     for the routine's test state, e.g. "sysid-test-state-mechanism". Defaults to the name of
     *     the subsystem if left null.
     */
    public SysIdRoutineMechanism(
            Consumer<Measure<Voltage>> drive,
            Consumer<SysIdRoutineLog> log,
            Subsystem subsystem,
            String name) {
        this.drive = drive;
        this.log = log != null ? log : l -> {};
        this.subsystem = subsystem;
        this.name = name != null ? name : "";
    }

    /**
     * Create a new mechanism specification for a SysId routine. Defaults the mechanism name to the
     * subsystem name.
     *
     * @param drive Sends the SysId-specified drive signal to the mechanism motors during test
     *     routines.
     * @param log Returns measured data of the mechanism motors during test routines. To return
     *     data, call `motor(string motorName)` on the supplied `SysIdRoutineLog` instance, and then
     *     call one or more of the chainable logging handles (e.g. `voltage`) on the returned
     *     `MotorLog`. Multiple motors can be logged in a single callback by calling `motor`
     *     multiple times.
     * @param subsystem The subsystem containing the motor(s) that is (or are) being characterized.
     *     Will be declared as a requirement for the returned test commands. The subsystem's `name`
     *     will be appended to the log entry title for the routine's test state, e.g.
     *     "sysid-test-state-subsystem".
     */
    public SysIdRoutineMechanism(
            Consumer<Measure<Voltage>> drive, Consumer<SysIdRoutineLog> log, Subsystem subsystem) {
        this(drive, log, subsystem, null);
    }
}
