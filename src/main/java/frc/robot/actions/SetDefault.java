package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.scheduling.actions.Actions;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

import javax.swing.plaf.basic.BasicSliderUI;

public class SetDefault extends ActionBase {
    private Arm arm;
    private ShooterSystem shooter;
    private Intake intake;

    public SetDefault(Arm arm, ShooterSystem shooter, Intake intake){
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        requires(intake,shooter);
    }
    @Override
    public void initialize(ActionControl control) {
        arm.doNotBaseOnLimelightDetection();
        arm.setNotAmp();
        arm.setSetPointAngle(Arm.DEF_ANGLE);
    }

    @Override
    public void execute(ActionControl control) {

    }

    @Override
    public void end(FinishReason reason) {

    }
}
