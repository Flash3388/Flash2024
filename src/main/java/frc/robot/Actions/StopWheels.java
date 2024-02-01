package frc.robot.Actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ShooterSystem;

public class StopWheels extends ActionBase {
    private ShooterSystem shooter;

    public StopWheels(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl actionControl) {
        shooter.setPIDStop();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
