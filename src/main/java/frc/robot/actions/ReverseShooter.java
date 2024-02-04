package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subSystems.ShooterSystem;

public class ReverseShooter extends ActionBase {
    private ShooterSystem shooter;

    public ReverseShooter(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl actionControl) {
        shooter.reverse();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
