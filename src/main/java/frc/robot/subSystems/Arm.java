package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

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

    private static final double HIGH_ANGLE = 40;
    private static final double AMP_ANGLE = 20;

    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    private static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 120;
    private final Timer timer = new Timer();


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
// 1. What happens if the PID reaches the required angle? 
//      Will the arm start to fall down because the motor is receiving 0 velocity?
//      What will keep the arm in place

//////////////////////

    public void moveToAngle(double angle){
        master.set(pid.applyAsDouble(getAngle2Target(), angle));
        timer.reset();
        timer.start();
    }

    public void moveUp(){
        master.set(SLOW_SPEED_UP);
    }

    public void moveDown(){
        master.set(SLOW_SPEED_DOWN);
    }

    // in case measurements do not work
    public void moveToSpeakerFixed(){
        moveToAngle(HIGH_ANGLE);

    }

    public void moveToAmp(){
        moveToAngle(AMP_ANGLE);
    }

    public void moveTofloor(){
        moveToAngle(AMP_ANGLE);
    }
  
    // When we reach the floor we would like this to be called 
    // so that the motor will not overheat 
    public void stopMotors(){
        master.stopMotor();
        timer.stop();
    }

    public void stayOnAngle(){
        moveToAngle(getAngle2Target());
    }

    public void checkTimer(){
        if (timer.hasElapsed(MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS)){
            stopMotors();
        }
    }

   

///////////////////////

    public void angleReset(){
        encoder.reset();
    }

    public double getAngle2Target(){
        return (encoder.getAbsolutePosition()- encoder.getPositionOffset()) * 360;
    }

    public void pidReset(){
        pid.reset();
    }

    public void stop(){ master.set(SPEED);}
    public void print(){
        SmartDashboard.putNumber("offSet", encoder.getAbsolutePosition());
    }
}
