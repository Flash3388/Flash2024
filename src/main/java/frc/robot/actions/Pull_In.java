package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Intake;

public class Pull_In extends ActionBase {

    private Intake intake;
    public Pull_In(Intake intake){
        this.intake = intake;
        requires(intake);
    }
    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        if(!intake.isIN())
            intake.pullIn();
        else
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        intake.stop();
    }
}
