package frc.robot.actions;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class ShooterSpeaker extends ActionBase {
    private ShooterSystem shooter;
    private Intake intake;
    private Arm arm;

    private static final double TIME = 2;
    private Clock clock;
    private Time time;


    public ShooterSpeaker(ShooterSystem shooter, Intake intake, Arm arm){
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;


        this.clock = RunningRobot.getControl().getClock();

        requires(shooter, intake, arm);

        configure().setName("ShooterSpeaker").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
        time = Time.INVALID;
    }

    @Override
    public void execute(ActionControl actionControl) {
        SmartDashboard.putBoolean("SpeakerBool", true);
        if (arm.isStabilizedAtTargetedPosition())
            shooter.shootSpeaker();

        if (shooter.gotToTarget(ShooterSystem.SPEED_TARGET_SPEAKER))
            intake.shoot();


       /* if (!intake.isIN()) {

            if (time.isValid()) {
                if(time.before(clock.currentTime()))
                    actionControl.finish();
            } else {
                time = clock.currentTime().add(Time.seconds(TIME));
            }
        }
        else{
            time = Time.INVALID;
        }*/

        if (!intake.isIN())
            actionControl.finish();
    }


    @Override
    public void end(FinishReason reason) {
        shooter.stop();
        intake.stop();
        arm.setPositioningNotControlled();
    }
}
