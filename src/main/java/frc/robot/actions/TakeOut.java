package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;

public class TakeOut extends ActionBase {
    private Intake intake;
    private Arm arm;

    public TakeOut(Intake intake, Arm arm){
        this.intake = intake;
        this.arm = arm;
        requires(intake, arm);
    }

    @Override
    public void initialize(ActionControl control) {//check if up
        arm.setSetPointAngle(5);
    }

    @Override
    public void execute(ActionControl control) {
        if(intake.isIN()) {
            this.intake.takeOut();
        }
        else control.finish();
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    }
}
