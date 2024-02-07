package frc.robot.subSystems;

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
        pid.setOutputLimit(0,1);
        pid.setTolerance(ERROR, Time.milliseconds(500));
        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);

        master.setIdleMode(CANSparkBase.IdleMode.kCoast);
        follower.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }


    public void shootSpeaker(){
        double speed = pid.applyAsDouble(getSpeed(),SPEED_TARGET_SPEAKER);

        SmartDashboard.putNumber("Speed", -speed);
        SmartDashboard.putBoolean("isStopped",false);
        this.master.set(-speed);
    }

    public void shootAmp(){
        double speed = pid.applyAsDouble(getSpeed(),SPEED_TARGET_AMP);

        SmartDashboard.putNumber("Speed", -speed);
        SmartDashboard.putBoolean("isStopped",false);
        this.master.set(-speed);
    }

    public void reverse(){
        double speed = pid.applyAsDouble(getSpeed(),SPEED_TARGET_SPEAKER);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putBoolean("isStopped",false);
        this.master.set(speed);
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
        SmartDashboard.putBoolean("AMPBool", false);
        SmartDashboard.putBoolean("SpeakerBool", false);
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