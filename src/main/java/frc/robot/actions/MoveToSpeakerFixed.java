package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.jmath.ExtendedMath;
import frc.robot.subSystems.Arm;


public class MoveToSpeakerFixed extends ActionBase {

    private Arm arm;

    public MoveToSpeakerFixed(Arm arm){
        this.arm = arm;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
    }

    @Override
    public void execute(ActionControl control) {
        arm.moveToSpeakerFixed();
        if(ExtendedMath.constrained(arm.getAngle(), Arm.SPEAKER_ANGLE - 2, Arm.SPEAKER_ANGLE +2))
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {

    }
}
