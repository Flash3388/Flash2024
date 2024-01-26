package actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import subSystems.Arm;


public class SetAngle extends ActionBase {

    private Arm arm;

    public SetAngle(Arm arm){
        this.arm = arm;

        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
    }

    @Override
    public void execute(ActionControl control) {
        arm.backwards();    //Will change following a condition to check the direction
        arm.forward();    //"                                                       "
    }

    @Override
    public void end(FinishReason reason) {

        arm.stop();
    }
}
