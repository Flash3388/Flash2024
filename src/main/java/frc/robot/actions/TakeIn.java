package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class TakeIn extends ActionBase {
    private Intake intake;
    private Arm arm;
    private ShooterSystem shooterSystem;

    public TakeIn(Intake intake , Arm arm, ShooterSystem shooterSystem){
        this.intake = intake;
        this.arm = arm;
        this.shooterSystem = shooterSystem;
        requires(intake);
    }
    @Override
    public void initialize(ActionControl control) {
        arm.setSetPointAngle(5);
    }

    @Override
    public void execute(ActionControl control) {
        if(arm.getArmPosition() <= 5) arm.setPositioningNotControlled();
        if(!this.intake.isIN())
        {
            this.intake.takeIn();
        }
        else{
            control.finish();
        }
    }

    @Override
    public void end(FinishReason reason) {
        arm.setSetPointAngle(Arm.DEF_ANGLE);
        //arm.setSetPointAngle(Arm.FLOOR_ANGLE);
        shooterSystem.moveDefault(intake.isIN());
        this.intake.stop();
    }
}
