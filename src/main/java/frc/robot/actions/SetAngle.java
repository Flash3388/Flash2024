package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.Requirement;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;


public class SetAngle extends ActionBase {

    private Arm arm;
    private boolean high;
    private boolean low;
    public SetAngle(Arm arm, boolean high, boolean low){   //high true
        this.arm = arm;
        this.high = true;
        this.low = false;

        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
        arm.angleReset();
    }

    @Override
    public void execute(ActionControl control) {
        if(high){
            arm.moveHigh();
        }
        else{
            arm.moveLow();
        }
    }

    @Override
    public void end(FinishReason reason) {

        arm.stop();
    }
}
