package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.scheduling.actions.Actions;
import frc.robot.subSystems.*;

import javax.naming.ldap.Control;
import javax.swing.plaf.basic.BasicSliderUI;

public class SetDefault extends ActionBase {
    private Arm arm;
    private ShooterSystem shooter;
    private Intake intake;
    private Limelight limelight;
    private Climb climb;

    public SetDefault(Arm arm, ShooterSystem shooter, Intake intake, Limelight limelight, Climb climb){
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.limelight = limelight;
        this.climb = climb;
        configure().setName("setDefault").save();
        requires(intake,shooter,limelight, climb);
    }
    @Override
    public void initialize(ActionControl control) {
        arm.doNotBaseOnLimelightDetection();
        arm.setNotAmp();
        arm.setSetPointAngle(Arm.DEF_ANGLE);
        Arm.isSetToClimbing = false;
        shooter.moveDefault();
        climb.goDown();
        control.finish();
    }

    @Override
    public void execute(ActionControl control) {

    }

    @Override
    public void end(FinishReason reason) {

    }
}
