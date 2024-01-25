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
    private final double kf = 0;
    private final double kd = 0;
    private final double ki= 0;
    private final double kp = 0;
    private PidController pid;
    private double ERROR = 0.05;
    private static final double SPEED_POINT = 0.5; // TODO: Figure ideal value for this variable.

    public ShooterSystem(CANSparkMax master, CANSparkMax follower){
        this.master = master;
        this.follower = follower;
        this.encoder = (RelativeEncoder) master.getEncoder();
        this.follower.follow(master, true); // TODO: Make sure this needs to be inverted.
        this.pid = new PidController(RunningRobot.getControl().getClock(),
                ()-> SmartDashboard.getNumber("KP", kp),
                ()-> SmartDashboard.getNumber("KI", ki),
                ()-> SmartDashboard.getNumber("KD", kd),
                ()-> SmartDashboard.getNumber("KF", kf));
        pid.setOutputLimit(-0.7,0.7);
        pid.setTolerance(ERROR, Time.milliseconds(500));
        SmartDashboard.putNumber("KP", kp);
        SmartDashboard.putNumber("KI", ki);
        SmartDashboard.putNumber("KD", kd);
        SmartDashboard.putNumber("KF", kf);
    }

    public void shoot(){
        this.master.set(pid.applyAsDouble(getSpeed(),SPEED_POINT));
    }

    public void reverse(){
        this.master.set(-pid.applyAsDouble(getSpeed(),SPEED_POINT));
    }

    public double getSpeed(){
        return this.encoder.getVelocity();
    }

    public void stop(){
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
   \ \/   /   )   \( `^^^
    \   \/    (    )
     \   )     )  /
Hey   ) /__    | (__
     (___)))   (__)))
 */