package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Climb;

public class ClimbDown extends ActionBase {
    private Climb climbMotor;

    public ClimbDown(Climb climb){
        this.climbMotor = climb;
        requires(climb);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        climbMotor.goDown();
        if(climbMotor.isReversePressed())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {

    }
}
