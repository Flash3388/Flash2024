package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.Requirement;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;


public class AutoAim extends ActionBase {

    private Arm arm;

    public AutoAim(Arm arm){
        this.arm = arm;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
    }

    @Override
    public void execute(ActionControl control) {
        //arm.autoAim();
    }

    @Override
    public void end(FinishReason reason) {
    }
}
