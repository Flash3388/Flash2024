package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
    // for motor PID
    private static  double KP = 0.0099; // tune this
    private static  double KI = 0.0004; // tune this
    private static  double KD = 0.000001; // tune this
    private static  double I_ZONE = 15; // tune this

    // for trapezoid profile
    private static  double MAX_VELOCITY = 1; // meters per second
    private static  double MAX_ACCELERATION = 1; // meters per second squared



    private final CANSparkMax master;
    private final CANSparkMax follower;
    private final DutyCycleEncoder absEncoder;
    private PidController pid;


    private static final double STABLE_ERROR = 1;
    private static final double STABLE_OUTPUT = 0.1;

    // Other Constants
    public static final double SPEAKER_ANGLE = 30;
    public static final double AMP_ANGLE_FROM_SHOOTER = 100;
    public static final double AMP_ANGLE_FROM_INTAKE = 53;
    public static final double FLOOR_ANGLE = -6.5; // the floor angle
    public static final double DEF_ANGLE = 20.0;


    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    public static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 60.0;

    private static final double GEAR_RATIO = 1/70.0;

    public static boolean isSetToAMP = false;

    private double setPointAngle;
    public static boolean BASED_ON_LIMELIGHT_DETECTION = false;

    public Arm(CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.master = master;
        this.follower = follower;
        this.absEncoder = encoder;

        this.follower.follow(this.master, true);
        this.master.follow(CANSparkBase.ExternalFollower.kFollowerDisabled, 0); // this is to make sure the master won't follow anyone

        absEncoder.setPositionOffset(81.79668 / 360);

        SmartDashboard.putNumber("ARM P Gain", KP);
        SmartDashboard.putNumber("ARM I Gain", KI);
        SmartDashboard.putNumber("ARM D Gain", KD);
        SmartDashboard.putNumber("ARM IZ Zone", I_ZONE);

        SmartDashboard.putNumber("ARM Max Velocity", MAX_VELOCITY);
        SmartDashboard.putNumber("ARM Max Acceleration", MAX_ACCELERATION);



        pid = PidController.newNamedController("drive", KP, KI, KD, 0);
        pid.setIZone(I_ZONE);

        setPointAngle = DEF_ANGLE;
        SmartDashboard.putNumber("set point A", DEF_ANGLE);


        master.setSmartCurrentLimit(80);
        follower.setSmartCurrentLimit(80);


        master.setIdleMode(CANSparkBase.IdleMode.kBrake);
        follower.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    public void moveToAngle(double angle){
        if(angle - getArmPosition() > 60)
            angle = (angle+getArmPosition())/2;

        if(getArmPosition() <= 0){
            angle = 10;
        }


        double speed = pid.applyAsDouble(getArmPosition(), angle) ;
        speed = ExtendedMath.constrain(speed, -0.5, 0.5);

        if(( getArmPosition() - angle) > 30)
            speed = speed / 3;



        if(getArmPosition() > 80 && speed > 0)
            speed = speed / 2;

        master.set(speed);
    }


    // When we reach the floor we would like this to be called
    // so that the motor will not overheat
    public void stopMotors(){
        master.stopMotor();
    }


    public double getArmPosition(){
        return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
    }

    public void resetPID(){
        pid.reset();
    }

    public void print(){
        SmartDashboard.putNumber("offSet", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ABS Arm position", getArmPosition());
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

        double maxV = SmartDashboard.getNumber("ARM Max Velocity", MAX_VELOCITY);
        double maxA = SmartDashboard.getNumber("ARM Max Acceleration", MAX_ACCELERATION);

        if (iz != I_ZONE) {
            I_ZONE = iz;
            pid.setIZone(iz);
        }
    }

    public double getSetPointAngle() {
        return setPointAngle;
    }

    public void setSetPointAngle(double setPointAngle) {
      //  SmartDashboard.putNumber("set point A", setPointAngle);
        if(setPointAngle == Double.MIN_VALUE)
            this.setPointAngle = setPointAngle;
        else {
            setPointAngle = ExtendedMath.constrain(setPointAngle, FLOOR_ANGLE, 100);
            this.setPointAngle = setPointAngle;
        }
    }

    public void setPositioningNotControlled() {
        setSetPointAngle(Double.MIN_VALUE);
        SmartDashboard.putNumber("set point A", Double.MIN_VALUE);
    }

    public boolean isPositioningControlled() {
        return setPointAngle != Double.MIN_VALUE;
    }

    public boolean isStabilizedAtTargetedPosition() {
        if (!isPositioningControlled()) {
            return false;
        }

        return ExtendedMath.constrained(getArmPosition(), setPointAngle - STABLE_ERROR, setPointAngle + STABLE_ERROR) ;
             //   && Math.abs(master.getAppliedOutput()) < STABLE_OUTPUT;
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

    public void setNotAmp(){
        isSetToAMP = false;
    }

    public void setYesAmp(){
        isSetToAMP = true;
    }
    public boolean isSetToAMP(){
        return isSetToAMP;
    }
    public void baseOnLimelightDetection(){
        BASED_ON_LIMELIGHT_DETECTION = true;
    }
    public void doNotBaseOnLimelightDetection(){
        BASED_ON_LIMELIGHT_DETECTION = false;
    }
    public boolean isBasedOnLimelightDetection(){
        return BASED_ON_LIMELIGHT_DETECTION;
    }


}
