package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class ShootToSpeaker extends ActionBase {
    private ShooterSystem shooterSystem;
    private Arm arm;
    private Intake intake;

    public ShootToSpeaker(ShooterSystem shooterSystem, Arm arm, Intake intake){
        this.shooterSystem = shooterSystem;
        this.arm = arm;
        this.intake = intake;
        requires(shooterSystem);
    }

    @Override
    public void initialize(ActionControl control) {
        shooterSystem.resetPID();
        arm.setNotAmp();
        arm.setSetPointAngle(Arm.SPEAKER_ANGLE);
    }

    @Override
    public void execute(ActionControl control) {
        shooterSystem.shootSpeaker();
        if(!intake.isIN())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {

    }
}
