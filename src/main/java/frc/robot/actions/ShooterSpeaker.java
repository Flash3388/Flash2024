package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class ShooterSpeaker extends ActionBase {
    private ShooterSystem shooter;
    private Intake intake;


    public ShooterSpeaker(ShooterSystem shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        requires(shooter, intake);

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
        if(shooter.gotToTarget(ShooterSystem.SPEED_TARGET_SPEAKER))
            intake.shoot();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
        intake.stop();

    }
}
