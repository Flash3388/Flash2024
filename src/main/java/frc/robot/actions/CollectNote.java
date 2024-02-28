package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Swerve;

public class CollectNote extends ActionBase {
    private Swerve swerve;

    public CollectNote(Swerve swerve){
        this.swerve = swerve;
        requires(swerve);
    }

    @Override
    public void initialize(ActionControl control) {
        swerve.resetWheels();
    }

    @Override
    public void execute(ActionControl control) {
        swerve.drive(-2, 0, 0);
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
