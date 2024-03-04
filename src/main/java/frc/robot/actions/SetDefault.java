package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.scheduling.actions.Actions;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.ShooterSystem;

import javax.naming.ldap.Control;
import javax.swing.plaf.basic.BasicSliderUI;

public class SetDefault extends ActionBase {
    private Arm arm;
    private ShooterSystem shooter;
    private Intake intake;
    private Limelight limelight;

    public SetDefault(Arm arm, ShooterSystem shooter, Intake intake, Limelight limelight){
        this.arm = arm;
        this.intake = intake;
        this.shooter = shooter;
        this.limelight = limelight;
        configure().setName("setDefault").save();
        requires(intake,shooter,limelight);
    }
    @Override
    public void initialize(ActionControl control) {
        arm.doNotBaseOnLimelightDetection();
        arm.setNotAmp();
        //arm.setSetPointAngle(Arm.DEF_ANGLE);
        if(arm.isPositioningControlled())
            arm.setSetPointAngle(Arm.FLOOR_ANGLE);
        shooter.moveDefault(intake.isIN());
        control.finish();
    }

    @Override
    public void execute(ActionControl control) {

    }

    @Override
    public void end(FinishReason reason) {

    }
}
