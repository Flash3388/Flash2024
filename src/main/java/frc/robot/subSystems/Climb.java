package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

public class Climb extends Subsystem {
    private CANSparkMax motor;
    private static final double CLIMB_SPEED = -0.6;
    private static final double GO_DOWN = 0.2;
    private SparkLimitSwitch forwardLimit;
    private SparkLimitSwitch reverseLimit;

    public Climb(CANSparkMax motor){
        this.motor = motor;
        this.forwardLimit = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        this.reverseLimit = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    }

    public void climb(){
        motor.set(CLIMB_SPEED);
    }

    public void goDown(){
        motor.set(GO_DOWN);
    }

    public void stop(){
        motor.stopMotor();
    }

    public boolean isForwardPressed(){
        return forwardLimit.isPressed();
    }

    public boolean isReversePressed(){
        return reverseLimit.isPressed();
    }
}
