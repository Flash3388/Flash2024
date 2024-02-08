package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subsystems.Intake;

public class TakeIn extends ActionBase {
    private Intake intake;

    public TakeIn( Intake intake){

        this.intake = intake;
        requires(intake);
       
    }
    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        if(!this.intake.isIN())
        {
            this.intake.takeIn();
        }
        else{control.finish();}
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    }
}
