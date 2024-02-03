package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import com.revrobotics.CANSparkMax;
import com.flash3388.flashlib.robot.control.PidController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {

    // Component
    private CANSparkMax master;
    private CANSparkMax follower;
    private DutyCycleEncoder absEncoder;
    private SparkPIDController pidController;
    private RelativeEncoder relEncoder;
    PowerDistribution powerDistribution = new PowerDistribution(PowerDistribution.kDefaultModule, PowerDistribution.ModuleType.kRev);


    // Smart Motion's Constants
    private static double KP = 0;
    private static double KI = 0;
    private static double KD = 0;
    private static double KF = 0;
    private static double KIZ = 0;
    private static double K_MAX_OUTPUT = 0.5;
    private static double K_MIN_OUTPUT = -0.5;
    private static double K_MAX_VELOCITY = 0;
    private static double K_MIN_VELOCITY = 0;
    private static double K_MAX_ACC = 0;
    private static final double ERROR = 0;

    // These are used for our positioning checks but not for the spark PID controller
    private static final double STABLE_ERROR = 0.5;
    private static final double STABLE_OUTPUT = 0.1;

    // Other Constants
    public static final double SPEAKER_ANGLE = 40; // todo: find the right angle
    public static final double AMP_ANGLE = 20; // todo: find the right angle
    public static final double FLOOR_ANGLE = -10;

    private static final double SLOW_SPEED_DOWN = -0.1;
    private static final double SLOW_SPEED_UP = 0.2;

    public static final double MOTOR_SAFEGUARD_TIMEOUT_IN_SECONDS = 120.0;

    private static final double GEAR_RATIO = 1/70.0; // todo: ask Ido about the gear ratio

    private double setPointAngle;

    public Arm(CANSparkMax master, CANSparkMax follower, DutyCycleEncoder encoder){
        this.master = master;
        this.follower = follower;
        this.absEncoder = encoder;
        this.relEncoder = master.getEncoder(); // Do I need to do the same to the follower?

        follower.follow(master, true);

        this.pidController = master.getPIDController();
      //pid.setOutputRampRate(0.01);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("ARM P Gain", KP);
        SmartDashboard.putNumber("ARM I Gain", KI);
        SmartDashboard.putNumber("ARM D Gain", KD);
        SmartDashboard.putNumber("ARM I Zone", KIZ);
        SmartDashboard.putNumber("ARM Feed Forward", KF);
        SmartDashboard.putNumber("ARM Max Output", K_MAX_OUTPUT);
        SmartDashboard.putNumber("ARM Min Output", K_MIN_OUTPUT);
        SmartDashboard.putNumber("ARM Max Velocity", K_MAX_VELOCITY);
        SmartDashboard.putNumber("ARM Min Velocity", K_MIN_VELOCITY);
        SmartDashboard.putNumber("ARM Max Acceleration", K_MAX_ACC);
        SmartDashboard.putNumber("ARM Allowed Closed Loop Error", ERROR);

        pidController.setP(KP);
        pidController.setI(KI);
        pidController.setD(KD);
        pidController.setFF(KF);
        pidController.setIZone(KIZ); //
        pidController.setOutputRange(K_MIN_OUTPUT, K_MAX_OUTPUT);

        pidController.setSmartMotionMaxVelocity(K_MAX_VELOCITY, 0);
        pidController.setSmartMotionMinOutputVelocity(K_MIN_VELOCITY, 0);
        pidController.setSmartMotionMaxAccel(K_MAX_ACC, 0);
        pidController.setSmartMotionAllowedClosedLoopError(ERROR, 0);

        this.absEncoder.setPositionOffset(85.5 / 360);
        this.relEncoder.setPosition((this.absEncoder.getAbsolutePosition() - this.absEncoder.getPositionOffset()) / GEAR_RATIO);

        master.setSmartCurrentLimit(80);
        follower.setSmartCurrentLimit(80);
    }

    public void moveToAngle(double angle){
        pidController.setReference(angle / 360 / GEAR_RATIO, CANSparkMax.ControlType.kSmartMotion);

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


    public double getCurrentAngle(){
        return relEncoder.getPosition() * GEAR_RATIO * 360;
    }

    public void pidReset(){
         // how can I reset the pidController?
    }

    public void print(){
        SmartDashboard.putNumber("offSet", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm's Angle", getCurrentAngle());
        SmartDashboard.putNumber("arm master output", master.getAppliedOutput());
        SmartDashboard.putNumber("arm follower output", follower.getAppliedOutput());
        SmartDashboard.putNumber("arm 18 amp", powerDistribution.getCurrent(18));
        SmartDashboard.putNumber("arm 19 amp", powerDistribution.getCurrent(19));

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);
    }

    public void changePidValues(){
        double p = SmartDashboard.getNumber("ARM P Gain", 0);
        double i = SmartDashboard.getNumber("ARM I Gain", 0);
        double d = SmartDashboard.getNumber("ARM D Gain", 0);
        double iz = SmartDashboard.getNumber("ARM I Zone", 0);
        double ff = SmartDashboard.getNumber("ARM Feed Forward", 0);
        double max = SmartDashboard.getNumber("ARM Max Output", 0);
        double min = SmartDashboard.getNumber("ARM Min Output", 0);
        double maxV = SmartDashboard.getNumber("ARM Max Velocity", 0);
        double minV = SmartDashboard.getNumber("ARM Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("ARM Max Acceleration", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != KP)) { pidController.setP(p); KP = p; }
        if((i != KI)) { pidController.setI(i); KI = i; }
        if((d != KD)) { pidController.setD(d); KD = d; }
        if((iz != KIZ)) { pidController.setIZone(iz); KIZ = iz; }
        if((ff != KF)) { pidController.setFF(ff); KF = ff; }
        if((max != K_MAX_OUTPUT) || (min != K_MIN_OUTPUT)) {
            pidController.setOutputRange(min, max);
            K_MIN_OUTPUT = min; K_MAX_OUTPUT = max;
        }
        if((maxV != K_MAX_VELOCITY)) { pidController.setSmartMotionMaxVelocity(maxV,0); K_MAX_VELOCITY = maxV; }
        if((minV != K_MIN_VELOCITY)) { pidController.setSmartMotionMinOutputVelocity(minV,0); K_MIN_VELOCITY = minV; }
        if((maxA != K_MAX_ACC)) { pidController.setSmartMotionMaxAccel(maxA,0); K_MAX_ACC = maxA; }
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

        return ExtendedMath.constrained(getCurrentAngle(), setPointAngle - STABLE_ERROR, setPointAngle + STABLE_ERROR) &&
                Math.abs(master.getAppliedOutput()) < STABLE_OUTPUT;
    }

    public boolean isAtBottom() {
        return setPointAngle == FLOOR_ANGLE &&
                ExtendedMath.constrained(getCurrentAngle(), FLOOR_ANGLE - STABLE_ERROR, FLOOR_ANGLE + STABLE_ERROR);
    }
}
