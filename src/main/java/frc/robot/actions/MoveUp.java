package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;


public class MoveUp extends ActionBase {

    private Arm arm;

    public MoveUp(Arm arm){
        this.arm = arm;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();

    }

    @Override
    public void execute(ActionControl control) {
        arm.moveUp();
    }

    @Override
    public void end(FinishReason reason) {
        arm.stayOnAngle();
    }
}
