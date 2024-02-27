package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.jmath.ExtendedMath;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Climb;

public class ClimbUp extends ActionBase {
    private Climb climbMotor;
    private Arm arm;
    private UsbCamera camera;
    private VideoSink videoSink;
    private static final double ERROR = 1;
    public ClimbUp(Climb climb, Arm arm){
        this.climbMotor = climb;
        this.arm = arm;
        //this.camera = camera;
        //this.videoSink = videoSink;
        requires(climb);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        if(ExtendedMath.constrained(arm.getArmPosition(), Arm.CLIMB_ANGLE - ERROR, Arm.CLIMB_ANGLE + ERROR))
        {
            climbMotor.climb();
        }
        else{
            climbMotor.stop();
        }

        if(climbMotor.isForwardPressed())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
       // videoSink.setSource(null);
    }
}
