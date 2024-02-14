package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;

public class TakeIn extends ActionBase {
    private Intake intake;
    private Arm arm;

    public TakeIn(Intake intake , Arm arm){
        this.intake = intake;
        this.arm = arm;
        requires(intake);
    }
    @Override
    public void initialize(ActionControl control) {
        arm.setSetPointAngle(Arm.FLOOR_ANGLE);
    }

    @Override
    public void execute(ActionControl control) {
        if(!this.intake.isIN())
        {
            this.intake.takeIn();
        }
        else{
            control.finish();
            arm.setSetPointAngle(20);
        }
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    }
}
