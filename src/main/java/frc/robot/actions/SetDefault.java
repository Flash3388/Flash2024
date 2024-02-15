package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class SetDefault extends ActionBase {
    private Arm arm;
    private ShooterSystem shooter;
    private Intake intake;

    public SetDefault(Arm arm, ShooterSystem shooter, Intake intake){
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        requires(arm,intake,shooter);
    }
    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        arm.setSetPointAngle(Arm.DEF_ANGLE);
        arm.setNotAmp();
    }

    @Override
    public void end(FinishReason reason) {

    }
}
