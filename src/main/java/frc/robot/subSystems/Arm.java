package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {



    // for motor PID
    private static  double KP = 0.0082; // tune this
    private static  double KI = 0.0027; // tune this
    private static  double KD = 0; // tune this
    private static  double I_ZONE = 0; // tune this

    // for trapezoid profile
    private static  double MAX_VELOCITY = 1; // meters per second
    private static  double MAX_ACCELERATION = 1; // meters per second squared


    // for feed forward
    // can use this to help tune
    private static  double STATIC_GAIN = 0; // volts // tune this
    private static  double GRAVITY_GAIN = 0.12; // volts // tune this
    private static  double VELOCITY_GAIN = 0.1; // volts * seconds / radians // tune this


    private final CANSparkMax master;
    private final CANSparkMax follower;
    private ArmFeedforward motorFeedForward;
    private final DutyCycleEncoder absEncoder;
    private ProfiledPIDController pid;


    private static final double STABLE_ERROR = 0.5;
    private static final double STABLE_OUTPUT = 0.1;

    // Other Constants
    public static final double SPEAKER_ANGLE = 40; // todo: find the right angle
    public static final double AMP_ANGLE = 20; // todo: find the right angle
    public static final double FLOOR_ANGLE = -10;

    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    public static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 120.0;

    private static final double GEAR_RATIO = 1/70.0;

    private double setPointAngle;

    public Arm(CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.master = master;
        this.follower = follower;
        this.absEncoder = encoder;

        this.follower.follow(this.master, true);
        this.master.follow(CANSparkBase.ExternalFollower.kFollowerDisabled, 0); // this is to make sure the master won't follow anyone

        motorFeedForward = new ArmFeedforward(STATIC_GAIN, GRAVITY_GAIN, VELOCITY_GAIN);

        absEncoder.setPositionOffset(81.79668 / 360);

        SmartDashboard.putNumber("ARM P Gain", KP);
        SmartDashboard.putNumber("ARM I Gain", KI);
        SmartDashboard.putNumber("ARM D Gain", KD);
        SmartDashboard.putNumber("ARM IZ Zone", I_ZONE);

        SmartDashboard.putNumber("ARM Static Gain", STATIC_GAIN);
        SmartDashboard.putNumber("ARM Gravity", GRAVITY_GAIN);
        SmartDashboard.putNumber("ARM Velocity Gain", VELOCITY_GAIN);

        SmartDashboard.putNumber("ARM Max Velocity", MAX_VELOCITY);
        SmartDashboard.putNumber("ARM Max Acceleration", MAX_ACCELERATION);

        SmartDashboard.putNumber("set point A", 20);


        pid = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        master.setSmartCurrentLimit(80);
        follower.setSmartCurrentLimit(80);
    }

    public void moveToAngle(double angle){
        double speed = pid.calculate(getArmPosition(), angle) ;
        speed = ExtendedMath.constrain(speed, -0.5, 0.5);

        TrapezoidProfile.State setPointT = pid.getSetpoint(); // the TrapezoidProfile calculate its setPoints
        double feedForward = motorFeedForward.calculate(Math.toRadians(setPointT.position), setPointT.velocity); // how we find the right feedForward

        SmartDashboard.putNumber("MOTOR Set Point", speed) ;
        SmartDashboard.putNumber("MOTOR Feed Forward", feedForward);

        master.set(speed + feedForward/12.0);

    }


    // When we reach the floor we would like this to be called
    // so that the motor will not overheat
    public void stopMotors(){
        master.stopMotor();
    }


    private double getArmPosition(){
        return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
    }

    public void resetPID(){
        pid.reset(getArmPosition());
    }

    public void print(){
        SmartDashboard.putNumber("offSet", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm's Angle", getArmPosition());
        SmartDashboard.putNumber("arm master output", master.getAppliedOutput());
        SmartDashboard.putNumber("arm follower output", follower.getAppliedOutput());

        SmartDashboard.putNumber("follower set velocity", follower.get());
        SmartDashboard.putNumber("master set velocity", master.get());


    }

    public void changePidValues(){
        double p = SmartDashboard.getNumber("ARM P Gain", KP);
        double i = SmartDashboard.getNumber("ARM I Gain", KI);
        double d = SmartDashboard.getNumber("ARM D Gain", KD);
        double iz = SmartDashboard.getNumber("ARM IZ Zone", I_ZONE);

        double staticGain = SmartDashboard.getNumber("ARM Static Gain", STATIC_GAIN);
        double gravityG = SmartDashboard.getNumber("ARM Gravity", GRAVITY_GAIN);
        double velocityG = SmartDashboard.getNumber("ARM Velocity Gain", VELOCITY_GAIN);

        double maxV = SmartDashboard.getNumber("ARM Max Velocity", MAX_VELOCITY);
        double maxA = SmartDashboard.getNumber("ARM Max Acceleration", MAX_ACCELERATION);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != KP)) { pid.setP(p); KP = p; }
        if((i != KI)) { pid.setI(i); KI = i; }
        if((d != KD)) { pid.setD(d); KD = d; }
        if((iz != I_ZONE)) { pid.setIZone(iz); I_ZONE = iz; }

        if ((maxV != MAX_VELOCITY) || (maxA != MAX_ACCELERATION)) {
            MAX_VELOCITY = maxV;
            MAX_ACCELERATION = maxA;
            pid.setConstraints(new TrapezoidProfile.Constraints(maxV, maxA));
        }

        if((staticGain != STATIC_GAIN) || (gravityG != GRAVITY_GAIN) || (velocityG != VELOCITY_GAIN)) {
            STATIC_GAIN = staticGain;
            GRAVITY_GAIN = gravityG;
            VELOCITY_GAIN = velocityG;
            motorFeedForward = new ArmFeedforward(STATIC_GAIN, GRAVITY_GAIN, VELOCITY_GAIN);
        }
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

    public boolean isPositioningControlled() {
        return setPointAngle != Double.MIN_VALUE;
    }

    public boolean isStabilizedAtTargetedPosition() {
        if (!isPositioningControlled()) {
            return true;
        }

        return ExtendedMath.constrained(getArmPosition(), setPointAngle - STABLE_ERROR, setPointAngle + STABLE_ERROR) &&
                Math.abs(master.getAppliedOutput()) < STABLE_OUTPUT;
    }

    public boolean isAtBottom() {
        return setPointAngle == FLOOR_ANGLE &&
                ExtendedMath.constrained(getArmPosition(), FLOOR_ANGLE - STABLE_ERROR, FLOOR_ANGLE + STABLE_ERROR);
    }

    // in case measurements do not work
    public void moveUp(){
        master.set(SLOW_SPEED_UP);
    }

    public void moveDown(){
        master.set(SLOW_SPEED_DOWN);
    }
}
