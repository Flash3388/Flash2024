package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

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
        return encoder.getAbsolutePosition() * 360;
    }

    public double speed(){
        return pid.applyAsDouble(getAngle2Target(), angle2Target);
    }
    public void move(){
        master.set(speed());
    }
    public void pidReset(){
        pid.reset();
    }

    public void stop(){ master.stopMotor();}
}
