package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.Requirement;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.jmath.ExtendedMath;
import frc.robot.subSystems.Arm;


public class MoveToFloor extends ActionBase {

    private Arm arm;

    public MoveToFloor(Arm arm){
        this.arm = arm;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
    }

    @Override
    public void execute(ActionControl control) {
        arm.moveTofloor();
        if(ExtendedMath.constrained(arm.getAngle2Target(), Arm.FLOOR_ANGLE - 2, Arm.FLOOR_ANGLE +2))
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        arm.stopMotors();
    }
}
