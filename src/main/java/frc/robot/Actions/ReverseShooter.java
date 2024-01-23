package frc.robot.Actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.Subsystems.ShooterSystem;

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
        shooter.move(-0.2);
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
