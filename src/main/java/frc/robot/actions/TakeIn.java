package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.Requirement;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subsystems.Intake;

import java.lang.module.ModuleDescriptor;

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
    this.intake.takeIn();
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    }
}
