package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Climb;

public class ClimbUp extends ActionBase {
    private Climb climbMotor;
    private Arm arm;
    public ClimbUp(Climb climb, Arm arm){
        this.climbMotor = climb;
        this.arm = arm;
        requires(climb);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        if(arm.isStabilizedAtTargetedPosition())
        {
            climbMotor.climb();
        }
        else{
            climbMotor.stop();
        }

        if(climbMotor.isForwardPressed())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        Arm.isSetToClimbing = false;
    }
}
