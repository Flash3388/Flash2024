package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.ShooterSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterAMP extends ActionBase {
    private ShooterSystem shooter;

    public ShooterAMP(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);

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
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
