package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootAMP extends ActionBase {
    private ShooterSystem shooter;
    private Arm arm;

    public ShootAMP(ShooterSystem shooter, Arm arm){
        this.shooter = shooter;
        this.arm = arm;
        requires(shooter);

       // configure().setName("ShooterAMP").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
        shooter.shootAmp();
        arm.setYesAmp();
        arm.setSetPointAngle(Arm.AMP_ANGLE_FROM_SHOOTER);
        control.finish();
    }

    @Override
    public void execute(ActionControl actionControl) {

    }

    @Override
    public void end(FinishReason reason) {
    }
}
