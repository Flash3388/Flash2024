package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Arm extends Subsystem {

    private CANSparkMax master;
    private CANSparkMax follower;
    private DutyCycleEncoder encoder;
    private PidController pid;
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KF = 0;
    private static final double ERROR = 2;
    private static final double LIMIT = 0.4;
    private static final double SPEED = 0.2;

    public static final double SPEAKER_ANGLE = 40;
    public static final double AMP_ANGLE = 20;
    public static final double FLOOR_ANGLE = -10;

    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    private static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 120.0;
    private final Timer timer = new Timer();

    public enum ArmPosition{
        Floor,
        Speaker,
        Amp
    };

    public Arm(CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.master = master;
        this.follower = follower;
        this.encoder = encoder;

        follower.follow(master, true);

        this.pid = PidController.newNamedController("ARM PID", KP, KI, KD, KF);

        pid.setTolerance(ERROR, Time.milliseconds(500));
        pid.setOutputLimit(LIMIT);

        encoder.setPositionOffset(85.5 / 360);
    }
// 1. What happens if the PID reaches the required angle? 
//      Will the arm start to fall down because the motor is receiving 0 velocity?
//      What will keep the arm in place

//////////////////////

    public void moveToAngle(double angle){
        master.set(pid.applyAsDouble(getAngle(), angle));
        timer.reset();
        timer.start();
    }

    // in case measurements do not work
    public void moveUp(){
        master.set(SLOW_SPEED_UP);
    }

    public void moveDown(){
        master.set(SLOW_SPEED_DOWN);
    }


    public void moveToSpeakerFixed(){
        moveToAngle(SPEAKER_ANGLE);

    }

    public void moveToAmp(){
        moveToAngle(AMP_ANGLE);
    }

    public void moveTofloor(){
        moveToAngle(FLOOR_ANGLE);
    }
  
    // When we reach the floor we would like this to be called 
    // so that the motor will not overheat 
    public void stopMotors(){
        master.stopMotor();
        timer.stop();
    }

    public void stayOnAngle(){
        moveToAngle(getAngle());
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

    public double getAngle(){
        return (encoder.getAbsolutePosition()- encoder.getPositionOffset()) * 360;
    }

    public void pidReset(){
        pid.reset();
    }

    public void stop(){ master.set(SPEED);}
    public void print(){
        SmartDashboard.putNumber("offSet", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm's Angle", getAngle());
    }


    public boolean AtAmp(){
        return ExtendedMath.constrained(getAngle(), AMP_ANGLE -2, AMP_ANGLE +2);
    }
    public boolean AtSpeaker(){
        return ExtendedMath.constrained(getAngle(), SPEAKER_ANGLE -2, SPEAKER_ANGLE +2);
    }
    public boolean AtFloor(){
        return ExtendedMath.constrained(getAngle(), FLOOR_ANGLE -2, FLOOR_ANGLE +2);
    }
}
