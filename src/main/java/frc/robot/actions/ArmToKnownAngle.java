package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;

public class ArmToKnownAngle extends ActionBase {
    private Arm arm;
    private Arm.ArmPosition armPosition;

    public ArmToKnownAngle(Arm arm, Arm.ArmPosition armPosition){
        this.arm = arm;
        this.armPosition = armPosition;
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        this.armPosition = armPosition;
    }

    @Override
    public void execute(ActionControl control) {
        switch (armPosition){
            case Amp:
                arm.moveToAmp();
                if(arm.AtAmp())
                    control.finish();
                break;

            case Speaker:
                arm.moveToSpeakerFixed();
                if(arm.AtSpeaker())
                    control.finish();
                break;

            case Floor:
                arm.moveTofloor();
                if(arm.AtFloor())
                    control.finish();
                break;
        }
    }

    @Override
    public void end(FinishReason reason) {

    }
}
