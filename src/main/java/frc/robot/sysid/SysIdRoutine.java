package frc.robot.sysid;

import com.flash3388.flashlib.control.Direction;
import com.flash3388.flashlib.scheduling.actions.Action;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import java.util.function.Consumer;

public class SysIdRoutine extends SysIdRoutineLog {

    private final SysIdRoutineConfig config;
    private final SysIdRoutineMechanism mechanism;
    private final Consumer<SysIdRoutineLog.State> recordState;

    public SysIdRoutine(String name, SysIdRoutineConfig config, SysIdRoutineMechanism mechanism) {
        super(name);
        this.config = config;
        this.mechanism = mechanism;
        recordState = config.recordState != null ? config.recordState : this::recordState;
    }

    public Action quasistatic(Direction direction) {
        return new QuasistaticTest(config, mechanism, recordState, this, direction);
    }

}
