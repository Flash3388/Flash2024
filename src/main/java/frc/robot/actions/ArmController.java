package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;

public class ArmController extends ActionBase {

    private Arm arm;

    private double lastSetPoint;
    private final Timer timer;
    public static boolean BASED_ON_LIMELIGHT_DETECTION = false;

    public ArmController(Arm arm) {
        this.arm = arm;
        this.timer = new Timer();
        requires(arm);
    }

    @Override
    public void initialize(ActionControl control) {
        lastSetPoint = arm.getSetPointAngle() - 1; // what if I stop this action with setPositioningNotControlled and then returns to it?
        // is it going to throw an error due to the minValue - 1?
    }

    @Override
    public void execute(ActionControl control) {

        if(arm.getSetPointAngle() != lastSetPoint){
            lastSetPoint = arm.getSetPointAngle();

            if (lastSetPoint != Double.MIN_VALUE) {
                timer.reset();
                timer.start();
                arm.resetPID();
            } else {
                timer.stop();
                arm.stopMotors();
            }
        }

        if (lastSetPoint != Double.MIN_VALUE) {
            if (timer.hasElapsed(Arm.MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS) || arm.isAtBottom()) {
                arm.setPositioningNotControlled();
            } else {
                arm.moveToAngle(lastSetPoint);
            }
        }

        SmartDashboard.putNumber("Time counted", timer.get());
        // todo: maybe support stall to manually maintain position
        // doing that with constant speed and updating the set point angle to the current angle
    }

    @Override
    public void end(FinishReason reason) {
        timer.stop();
    }
}
