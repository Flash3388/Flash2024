package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;

public class StayOnAngle extends ActionBase {
    private Arm arm;
    private double setPoint;
    public StayOnAngle(Arm arm){
        this.arm = arm;
        setPoint = arm.getAngle();
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        setPoint = arm.getAngle();
    }

    @Override
    public void execute(ActionControl control) {
        arm.moveToAngle(setPoint);
    }

    @Override
    public void end(FinishReason reason) {

    }
}
