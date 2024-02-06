package frc.robot.Subsystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.revrobotics.CANSparkBase;
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
    private static final double SPEED_TARGET_SPEAKER = 4000;
    private static final double SPEED_TARGET_AMP = 2000;

    public ShooterSystem(CANSparkMax master, CANSparkMax follower){
        this.master = master;
        this.follower = follower;
        this.encoder = (RelativeEncoder) master.getEncoder();
        this.follower.follow(master, false);
        this.pid = new PidController(RunningRobot.getControl().getClock(),
                ()-> {return SmartDashboard.getNumber("KP", KP);},
                ()-> {return SmartDashboard.getNumber("KI", KI);},
                ()-> {return SmartDashboard.getNumber("KD", KD); },
                ()-> {return SmartDashboard.getNumber("KF", KF);});
        pid.setOutputLimit(-0.002,1);
        pid.setTolerance(ERROR, Time.milliseconds(500));
        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);

        master.setIdleMode(CANSparkBase.IdleMode.kCoast);
        follower.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

    public double getPID(double SPEED_POINT){
        double speed = pid.applyAsDouble(getSpeed(),SPEED_POINT);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putBoolean("isStopped",false);
        return speed;
    }


    public void shootSpeaker(){
        this.master.set(-getPID(SPEED_TARGET_SPEAKER));
    }

    public void shootAmp(){
        this.master.set(-getPID(SPEED_TARGET_AMP));
    }

    public void reverse(){
        this.master.set(getPID(SPEED_TARGET_SPEAKER));
    }

    public double getSpeed(){
        SmartDashboard.putNumber("RPM", -this.encoder.getVelocity());
        return -this.encoder.getVelocity();
    }

    public void resetPID(){
        pid.reset();
    }

    public void stop(){
        SmartDashboard.putBoolean("isStopped",true);
        this.master.stopMotor();
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