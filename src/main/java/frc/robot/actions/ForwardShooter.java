package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import frc.robot.subsystems.ShooterSystem;

public class ForwardShooter extends ActionBase {
    private ShooterSystem shooter;

    public ForwardShooter(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl actionControl) {
        shooter.shoot();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
