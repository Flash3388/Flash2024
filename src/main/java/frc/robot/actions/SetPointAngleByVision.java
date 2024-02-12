package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;

public class SetPointAngleByVision extends ActionBase {

    private Limelight limelight;
    private Intake intake;
    private Arm arm;

    public SetPointAngleByVision(Limelight limelight, Intake intake, Arm arm){
        this.intake = intake;
        this.limelight = limelight;
        this.arm = arm;
        requires(limelight, intake, arm);
    }
    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        if(intake.isIN()) {
            double distance = limelight.getDistanceToTarget();

            double angle = -1.05 * Math.pow(distance, 2) + 11.2 * distance + 18.4;
            arm.setSetPointAngle(angle);
        }
    }

    @Override
    public void end(FinishReason reason) {
        arm.setPositioningNotControlled();
    }
}
