package frc.robot.subSystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSystem extends Subsystem {
    private CANSparkMax master;
    private CANSparkMax follower;
    private RelativeEncoder encoder;
    // TODO: Figure out values for the 4 variables below.
    private final double KP = 1e-7; // we need to find the value
    private final double KI = 1e-7;
    private final double KD = 0;

    private final double KF = 0;


    private SparkPIDController pid;
    public static final double SPEED_TARGET_SPEAKER = 3000;
    public static final double SPEED_TARGET_AMP = 2000;

    public ShooterSystem(CANSparkMax master, CANSparkMax follower){
        this.master = master;
        this.follower = follower;
        this.encoder = (RelativeEncoder) master.getEncoder();
        this.follower.follow(master, false);
        this.pid = master.getPIDController();
        SmartDashboard.putNumber("KP", KP);
        SmartDashboard.putNumber("KI", KI);
        SmartDashboard.putNumber("KD", KD);
        SmartDashboard.putNumber("KF", KF);

        pid.setP(KP);
        pid.setI(KI);
        pid.setD(KD);
        pid.setFF(KF);
        pid.setOutputRange(-1,1);
        master.setIdleMode(CANSparkBase.IdleMode.kCoast);
        follower.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }


    public void shootSpeaker(){
        //double speed = pid.applyAsDoubleapplyAsDouble(getSpeed(),SPEED_TARGET_SPEAKER);
        pid.setReference(SPEED_TARGET_SPEAKER, CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target Speaker",gotToTarget(SPEED_TARGET_SPEAKER));

    }

    public void shootAmp(){
        pid.setReference(SPEED_TARGET_AMP, CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target Amp",gotToTarget(SPEED_TARGET_AMP));
    }

    public void reverse(){
        pid.setReference(500, CANSparkBase.ControlType.kSmartVelocity);

        SmartDashboard.putNumber("Speed", getSpeed());
        SmartDashboard.putBoolean("isStopped",false);
        SmartDashboard.putBoolean("got to target reverse",gotToTarget(500));

    }

    public double getSpeed(){
        SmartDashboard.putNumber("RPM", this.encoder.getVelocity());
        return this.encoder.getVelocity();
    }

    public void resetPID(){
        //pid.reset();
    }

    public void stop(){
        SmartDashboard.putBoolean("isStopped",true);
        SmartDashboard.putBoolean("AMPBool", false);
        SmartDashboard.putBoolean("SpeakerBool", false);
        this.master.stopMotor();
    }
    public boolean gotToTarget(double rpmVal){
        return ExtendedMath.constrained(getSpeed(),rpmVal - 100, rpmVal +100);
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