package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subsystems.Intake;

public class TakeOut extends ActionBase {
    private Intake intake;

    public TakeOut(Intake intake){

        this.intake = intake;
        requires(intake);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
    this.intake.takeOut();
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    }
}
