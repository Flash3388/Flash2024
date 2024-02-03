package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private static final double LIMIT = 0.5;
    private static final double SPEED = 0.2;

    public static final double SPEAKER_ANGLE = 40;
    public static final double AMP_ANGLE = 20;
    public static final double FLOOR_ANGLE = -10;

    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    public static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 120.0;
    PowerDistribution powerDistribution = new PowerDistribution(PowerDistribution.kDefaultModule, PowerDistribution.ModuleType.kRev);



    private double setPointAngle;


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
        pid.setOutputRampRate(0.01);

        pid.setTolerance(ERROR, Time.milliseconds(500));
        pid.setOutputLimit(LIMIT);

        encoder.setPositionOffset(85.5 / 360);
        master.setSmartCurrentLimit(80);
        follower.setSmartCurrentLimit(80);
    }
// 1. What happens if the PID reaches the required angle? 
//      Will the arm start to fall down because the motor is receiving 0 velocity?
//      What will keep the arm in place

//////////////////////

    public void moveToAngle(double angle){
        double val = pid.applyAsDouble(getCurrentAngle(), angle);
        master.set(val);
    }

    // in case measurements do not work
    public void moveUp(){
        master.set(SLOW_SPEED_UP);
    }

    public void moveDown(){
        master.set(SLOW_SPEED_DOWN);
    }

  
    // When we reach the floor we would like this to be called 
    // so that the motor will not overheat 
    public void stopMotors(){
        master.stopMotor();
    }

///////////////////////

    public double getCurrentAngle(){
        return (encoder.getAbsolutePosition()- encoder.getPositionOffset()) * 360;
    }

    public void pidReset(){
        pid.reset();
    }

    public void print(){
        SmartDashboard.putNumber("offSet", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm's Angle", getCurrentAngle());
        SmartDashboard.putNumber("arm master output", master.getAppliedOutput());
        SmartDashboard.putNumber("arm follower output", follower.getAppliedOutput());
        SmartDashboard.putNumber("arm 18 amp", powerDistribution.getCurrent(18));
        SmartDashboard.putNumber("arm 19 amp", powerDistribution.getCurrent(19));
    }

    public double getSetPointAngle() {
        return setPointAngle;
    }

    public void setSetPointAngle(double setPointAngle) {
        this.setPointAngle = setPointAngle;
    }

    public void setPositioningNotControlled() {
        setSetPointAngle(Double.MIN_VALUE);
    }

    public boolean isStabilizedAtTargetedPosition() {
        return pid.isInTolerance(getCurrentAngle(), setPointAngle);
    }

    public boolean isAtBottom() {
        return setPointAngle == FLOOR_ANGLE && ExtendedMath.constrained(getCurrentAngle(), FLOOR_ANGLE - ERROR, FLOOR_ANGLE + ERROR);
    }
}
