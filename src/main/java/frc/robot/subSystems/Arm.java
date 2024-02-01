package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

    private double angle2Target;
    private CANSparkMax master;
    private CANSparkMax follower;
    private DutyCycleEncoder encoder;
    private PidController pid;
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KF = 0;
    private static final double ERROR = 1;
    private static final double LIMIT = 1;
    private static final double SPEED = 0.2;
    private static final double HIGH = 40;
    public static final double LOW = 20;


    public Arm(double angle2Target, CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.angle2Target = angle2Target;
        this.master = master;
        this.follower = follower;
        this.encoder = encoder;

        follower.follow(master, true);

        this.pid = PidController.newNamedController("PID", KP, KI, KD, KF);

        pid.setTolerance(ERROR, Time.milliseconds(500));
        pid.setOutputLimit(LIMIT);


    }

    public void angleReset(){
        encoder.reset();
    }

    public double getAngle2Target(){
        return (encoder.getAbsolutePosition()- encoder.getPositionOffset()) * 360;
    }

    public double SpeedHigh(){
        angle2Target = HIGH;
        return pid.applyAsDouble(getAngle2Target(), angle2Target);
    }
    public double SpeedLow(){
        angle2Target = LOW;
        return pid.applyAsDouble(getAngle2Target(), angle2Target);
    }
    public void moveHigh(){
        master.set(SpeedHigh());
    }
    public void moveLow(){
        master.set(SpeedLow());
    }
    public void pidReset(){
        pid.reset();
    }

    public void stop(){ master.set(SPEED);}
    public void print(){
        SmartDashboard.putNumber("offSet", encoder.getAbsolutePosition());
    }
}
