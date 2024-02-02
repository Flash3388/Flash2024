package frc.robot.actions;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import frc.robot.subSystems.Arm;


public class SetAngle extends ActionBase {

    private Arm arm;
    private boolean high;
    private boolean low;
    private static final double ALLOWED_ERROR = 2;
    private static final double TIME = 1;
    private Clock clock;
    private Time time;
    public SetAngle(Arm arm, boolean high, boolean low){   //high true
        this.arm = arm;
        this.high = true;
        this.low = false;

        this.clock = RunningRobot.getControl().getClock();

        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.pidReset();
        arm.angleReset();
    }

    @Override
    public void execute(ActionControl control) {
        if(high){
            arm.moveHigh();
        }
        else{
            arm.moveLow();
        }


        if(inRange()){
            if(time.isValid()){
                time.before(clock.currentTime());
            }
            else{
                time = clock.currentTime().add(Time.seconds(TIME));
            }
        }
        else{
            time = Time.INVALID;
        }
        control.finish();
    }

    public boolean inRange(){
        return ExtendedMath.constrained(arm.getAngle2Target(),-ALLOWED_ERROR, ALLOWED_ERROR);
    }

    @Override
    public void end(FinishReason reason) {
        arm.slow();
    }
}
