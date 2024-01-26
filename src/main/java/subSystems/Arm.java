package subSystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

    private double angle;
    private CANSparkMax master;
    private CANSparkMax follower;
    private DutyCycleEncoder encoder;
    private PidController pid;
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KF = 0;
    private static final double ERROR = 0;
    private static final double LIMIT = 0;


    public Arm(double angle, CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.angle = angle;
        this.master = master;
        this.follower = follower;
        this.encoder = encoder;

        follower.follow(master);

        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);

        this.pid = new PidController(RunningRobot.getControl().getClock(),
                ()-> {
                    return SmartDashboard.getNumber("KP", KP);
                },
                ()->{
                    return SmartDashboard.getNumber("KI", KI);
                },
                ()->{
                    return SmartDashboard.getNumber("KD", KD);
                },
                ()->{
                    return SmartDashboard.getNumber("KF", KF);
                });

        pid.setTolerance(ERROR, Time.milliseconds(500));
        pid.setOutputLimit(LIMIT);

    }

    public double getAngle(){
        return 5;  //Will change
    }

    public double speed(){
        return pid.applyAsDouble(getAngle(), angle);
    }
    public void move(){
        master.set(speed());
    }
    public void pidReset(){
        pid.reset();
    }

    public void stop(){ master.stopMotor();}
}
