package frc.robot.actions;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.ShooterSystem;

public class ShootSpeaker extends ActionBase {
    private ShooterSystem shooter;
    private Intake intake;
    private Arm arm;

    private static final double DELAY_BEFORE_FINISH_IN_SECONDS = 1;
    private Clock clock;
    private Time time;


    public ShootSpeaker(ShooterSystem shooter, Intake intake, Arm arm){
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;


        this.clock = RunningRobot.getControl().getClock();

        requires(shooter, intake);

        //configure().setName("ShooterSpeaker").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
        time = Time.INVALID;
    }

    @Override
    public void execute(ActionControl actionControl) {
        if(arm.isSetToAMP()){

            if(arm.getSetPointAngle() == Arm.AMP_ANGLE_FROM_INTAKE){
                shootBackwardToAMP();
            }
            else if(arm.getSetPointAngle() == Arm.AMP_ANGLE_FROM_SHOOTER)
            {
                shootForwardToAMP();
            }
        }

        else {
            if (arm.isStabilizedAtTargetedPosition())
                shooter.shootSpeaker();

            if (shooter.gotToTarget(ShooterSystem.SPEED_TARGET_SPEAKER))
                intake.shoot();
        }

        if (!intake.isIN()) {
            if (time.isValid()) {
                if(time.before(clock.currentTime()))
                    actionControl.finish();
            } else {
                time = clock.currentTime().add(Time.seconds(DELAY_BEFORE_FINISH_IN_SECONDS));
            }
        }

        else{
            time = Time.INVALID;
        }
    }

    private void shootForwardToAMP(){
        shooter.shootAmp();
        if (shooter.gotToTarget(ShooterSystem.SPEED_TARGET_AMP))
            intake.takeIn();
    }
    private void shootBackwardToAMP(){
        intake.takeOut();
        shooter.reverse();
    }

    @Override
    public void end(FinishReason reason) {
        shooter.stop();
        intake.stop();
        arm.setNotAmp();
       // arm.setSetPointAngle(Arm.DEF_ANGLE);
        arm.setSetPointAngle(Arm.FLOOR_ANGLE);

    }
}
