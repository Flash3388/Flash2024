package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class TakeOut extends ActionBase {
    private Intake intake;
    private ShooterSystem shooter;
    private Arm arm;

    public TakeOut(Intake intake, Arm arm, ShooterSystem shooter){
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        requires(intake, arm,shooter);
    }

    @Override
    public void initialize(ActionControl control) {//check if up
        if(arm.getArmPosition() < 10)
            arm.setSetPointAngle(10);
    }

    @Override
    public void execute(ActionControl control) {
        this.shooter.reverse();
        this.intake.takeOut();
    }

    @Override
    public void end(FinishReason reason) {
    this.intake.stop();
    shooter.moveDefault(intake.isIN());
    }
}
