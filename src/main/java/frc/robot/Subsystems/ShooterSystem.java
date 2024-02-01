package frc.robot.Subsystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSystem extends Subsystem {
    private CANSparkMax master;
    private CANSparkMax follower;
    private RelativeEncoder encoder;
    // TODO: Figure out values for the 4 variables below.
    private final double KF = 0;
    private final double KD = 0;
    private final double KI = 0;
    private final double KP = 0;
    private PidController pid;
    private double ERROR = 50;
    private static final double SPEED_POINT = 4000; // TODO: Figure ideal value for this variable.

    public ShooterSystem(CANSparkMax master, CANSparkMax follower){
        this.master = master;
        this.follower = follower;
        this.encoder = (RelativeEncoder) master.getEncoder();
        this.follower.follow(master, false);
        this.pid = new PidController(RunningRobot.getControl().getClock(),
                ()-> SmartDashboard.getNumber("KP", KP),
                ()-> SmartDashboard.getNumber("KI", KI),
                ()-> SmartDashboard.getNumber("KD", KD),
                ()-> SmartDashboard.getNumber("KF", KF));
        pid.setOutputLimit(-0.4,0.4);
        pid.setTolerance(ERROR, Time.milliseconds(500));
        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);
    }

    public double getPID(){
        double speed = pid.applyAsDouble(getSpeed(),SPEED_POINT);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putBoolean("isStopped",false);
        return speed;
    }

    public void setPIDStop(){
        double speed = pid.applyAsDouble(getSpeed(),0);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putBoolean("GOT TO HEREEEEEEEEEEEEEEE", true);
        SmartDashboard.putBoolean("isStopped",true);
        this.master.set(speed);
    }


    public void shoot(){
        this.master.set(-getPID());
    }

    public void reverse(){
        this.master.set(getPID());
    }

    public double getSpeed(){
        SmartDashboard.putNumber("RPM", this.encoder.getVelocity());
        return this.encoder.getVelocity();
    }
    public void resetPID(){
        pid.reset();
    }

    public void stop(){
        SmartDashboard.putBoolean("isStopped",true);
        this.master.set(0.0);
    }
}


/*
HELLO TOM MEET THE CODE CAT.
     _
    / )
   / /
  / /               /\
 / /     .-```-.   / ^`-.
 \ \    /       \_/  (|) `o
  \ \  /   .-.   \\ _  ,--'
   \ \/   /   )   \( __^^^
    \   \/    (    )
     \   )     )  /
Hey   ) /__    | (__
     (___)))   (__)))
 */