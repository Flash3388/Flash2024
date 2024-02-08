package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterAMP extends ActionBase {
    private ShooterSystem shooter;
    private Intake intake;

    public ShooterAMP(ShooterSystem shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        requires(shooter, intake);

        configure().setName("ShooterAMP").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
    }

    @Override
    public void execute(ActionControl actionControl) {
            SmartDashboard.putBoolean("AMPBool", true);
        shooter.shootAmp();
        if(shooter.gotToTarget(ShooterSystem.SPEED_TARGET_AMP))
            intake.takeIn();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
        intake.stop();
    }
}
