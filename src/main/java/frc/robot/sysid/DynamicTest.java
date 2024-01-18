package frc.robot.sysid;

import com.flash3388.flashlib.control.Direction;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.util.Map;
import java.util.function.Consumer;

import com.flash3388.flashlib.control.Direction;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Time;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.util.function.Consumer;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class DynamicTest extends ActionBase {
    private final SysIdRoutineConfig config;
    private final SysIdRoutineMechanism mechanism;
    private final Consumer<SysIdRoutineLog.State> recordState;
    private final SysIdRoutineLog log;
    private final Direction direction;
    private final Timer timer;
    private final SysIdRoutineLog.State state;
    private MutableMeasure<Voltage> outputVolts;

    public DynamicTest(SysIdRoutineConfig config,
                       SysIdRoutineMechanism mechanism,
                       Consumer<SysIdRoutineLog.State> recordState,
                       SysIdRoutineLog log,
                       Direction direction) {
        this.config = config;
        this.mechanism = mechanism;
        this.recordState = recordState;
        this.log = log;
        this.direction = direction;

        this.timer = new Timer();

        state = Map.ofEntries(
                Map.entry(Direction.FORWARD, SysIdRoutineLog.State.kDynamicForward),
                Map.entry(Direction.BACKWARD, SysIdRoutineLog.State.kDynamicReverse))
                .get(direction);


        outputVolts = mutable(Volts.of(0));

        requires(mechanism.subsystem);

        configure()
                .setName("sysid-" + state + "-" + mechanism.name)
                .setTimeout(Time.seconds(config.timeout.in(Second)))
                .save();
    }

    @Override
    public void initialize(ActionControl control) {
        outputVolts = outputVolts.mut_replace(config.stepVoltage.in(Volts) * direction.sign(), Volts);
    }

    @Override
    public void execute(ActionControl control) {
        mechanism.drive.accept(outputVolts);
        mechanism.log.accept(log);
        recordState.accept(state);
    }

    @Override
    public void end(FinishReason reason) {
        mechanism.drive.accept(Volts.of(0));
        recordState.accept(SysIdRoutineLog.State.kNone);
    }
}

