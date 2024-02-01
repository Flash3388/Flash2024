package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.Requirement;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.jmath.ExtendedMath;
import frc.robot.subSystems.Arm;


public class MoveToAmp extends ActionBase {

    private Arm arm;

    public MoveToAmp(Arm arm){
        this.arm = arm;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
    }

    @Override
    public void execute(ActionControl control) {
        arm.moveToAmp();
        if(ExtendedMath.constrained(arm.getAngle2Target(), Arm.AMP_ANGLE - 2, Arm.AMP_ANGLE +2))
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
    }
}
