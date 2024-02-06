package frc.robot.Actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ShooterSystem;

public class ShooterSpeaker extends ActionBase {
    private ShooterSystem shooter;

    public ShooterSpeaker(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);

        configure().setName("ShooterSpeaker").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
    }

    @Override
    public void execute(ActionControl actionControl) {
        SmartDashboard.putBoolean("SpeakerBool", true);
        shooter.shootSpeaker();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
    }
}
