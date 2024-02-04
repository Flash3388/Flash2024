package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {

    // this example combines SparkMax's built-in PID position controller.
    // with WPI's Feed-Forward control and TrapezoidProfile for integrated motion profiling and "smart" feed-forward.
    // this basically provides a sort of "Smart-Motion" by integrating a velocity-profile, feedback and feed-forward control.

    // The PID constants will need tuning, this can be done per usual.
    // The Feed-Forward control will also need tuning to provide optimal control. Tune those, while
    // tuning the PID. mostly touch the GRAVITY_GAIN and VELOCITY_GAIN. They should help compensate for gravity.
    // Be careful with using VELOCITY_GAIN, as it would affect the velocity of the arm.
    // No need to touch STATIC_GAIN really, but if you see that the arm misses the setpoint
    //  by a constant amount, then it
    // would be beneficial to modify it just a bit. Values would generally be 0-2, but may exceed it a bit.
    // See this for example of feed-forward and feed-back control combined.
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html#combined-feedforward-and-feedback-control
    // See this to read more on ArmFeedForward
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html
    //
    // TrapezoidProfile provides the motion profiling component. It will allow limiting the velocity and acceleration
    // of the arm. To combine it with the motor PID controller, we basically slowly move the arm between setpoints
    // until we reach the wanted setpoint, instead of directly moving to the wanted setpoint. This allows limiting
    // the motion of the system and keeping it more stable.
    // See more on this here
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
    // The motion for it is controlled by MAX_VELOCITY and MAX_ACCELERATION, these should be modified if you want
    // to increase the speed of the arm, but carefully as to not be too fast. This goes doubly for MAX_ACCELERATION
    // as it will allow increased acceleration for the arm and thus cause it to accelerate quicly.

    // for motor PID
    private static  double KP = 0; // tune this
    private static  double KI = 0; // tune this
    private static  double KD = 0; // tune this
    private static  double I_ZONE = 0; // tune this

    // for trapezoid profile
    private static  double MAX_VELOCITY = 1; // meters per second
    private static  double MAX_ACCELERATION = 1; // meters per second squared

    // for feed forward
    // can use this to help tune

    private static  double STATIC_GAIN = 0; // volts // tune this
    private static  double GRAVITY_GAIN = 1; // volts // tune this
    private static  double VELOCITY_GAIN = 0; // volts * seconds / radians // tune this

    private static final double LOOP_TIME_SECONDS = 0.02; // this is a global constant for the robot code
    private static final double GEAR_RATIO = 1.0/70.0  ; // driver / driven
    private static final int PID_SLOT = 0; // default slot to use

    private final CANSparkMax motor;
    private final CANSparkMax follower;
    private final SparkPIDController motorPid;
    private final TrapezoidProfile motorPositionProfile;
    private final ArmFeedforward motorFeedForward;
    private final RelativeEncoder relativeEncoder1;
    private final RelativeEncoder relativeEncoder2;
    private final DutyCycleEncoder absEncoder;

    private TrapezoidProfile.State currentMotorSetPoint;
    private TrapezoidProfile.State endMotorSetPoint;


    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        follower = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
        follower.follow(motor, true);
        motorPid = motor.getPIDController();
        motorPositionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        motorFeedForward = new ArmFeedforward(STATIC_GAIN, GRAVITY_GAIN, VELOCITY_GAIN);
        relativeEncoder1 = motor.getEncoder();
        relativeEncoder2 = follower.getEncoder();



        absEncoder = new DutyCycleEncoder(9);
      //  absEncoder.setPositionOffset(81.79668 / 360);
        absEncoder.setPositionOffset(81.79668);

     //   relativeEncoder.setPosition((absEncoder.getAbsolutePosition() - this.absEncoder.getPositionOffset()) / GEAR_RATIO /360);
        relativeEncoder1.setPosition((absEncoder.getAbsolutePosition() - this.absEncoder.getPositionOffset()) / GEAR_RATIO /360);
        relativeEncoder2.setPosition((absEncoder.getAbsolutePosition() - this.absEncoder.getPositionOffset()) / GEAR_RATIO /360);

        motorPid.setP(KP);
        motorPid.setI(KI);
        motorPid.setD(KD);
        motorPid.setIZone(I_ZONE);
        motorPid.setFF(0); // don't use the motor's feed forward, we will supply our own
        motorPid.setOutputRange(-1, 1);
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        // set wanted final setpoint for the motor, 70 degrees, and stop at 0 velocity.
       // endMotorSetPoint = new TrapezoidProfile.State(50.0, 0);
        endMotorSetPoint = new TrapezoidProfile.State(9.0, 0);
        // set this to the current position (angle) and velocity (0).
        // for real robot, use the position supplied by the encoder
       // currentMotorSetPoint = new TrapezoidProfile.State(getArmPosition(), 0);
        currentMotorSetPoint = new TrapezoidProfile.State(getRelativeArmPosition(), 0);
    }

    @Override
    public void autonomousPeriodic() {
        // this will calculate our next setpoint for the motor,
        currentMotorSetPoint = motorPositionProfile.calculate(LOOP_TIME_SECONDS, currentMotorSetPoint, endMotorSetPoint);
        // currentMotorSetPoint.position == position [degrees] to move the arm to
        // currentMotorSetPoint.velocity == velocity [degrees/second] to move the arm at

        // this will provide a feed-forward
        double feedForward = motorFeedForward.calculate(Math.toRadians(currentMotorSetPoint.position), currentMotorSetPoint.velocity);
        // feedForward = volts to add to the motor output (if PID says output=5, then the real output will be 5+feedForward)

        // feed the info to the motor
        double positionForMotor = currentMotorSetPoint.position / 360.0 * GEAR_RATIO;
      /*  motorPid.setReference(
                positionForMotor,
                CANSparkBase.ControlType.kPosition,
                PID_SLOT,
                feedForward,
                SparkPIDController.ArbFFUnits.kVoltage); */

        changePidValues();
        SmartDashboard.putNumber("Arm position", getArmPosition());
       // SmartDashboard.putNumber("OFFSET", absEncoder.getAbsolutePosition() * 360);
        SmartDashboard.putNumber("OFFSET", getArmPosition());
        SmartDashboard.putNumber("GetRelativePosition", getRelativeArmPosition());

    }

    private double getArmPosition(){
        //return relativeEncoder.getPosition() * GEAR_RATIO  * 360;
      //  SmartDashboard.putNumber("rel position", relativeEncoder1.getPosition() * GEAR_RATIO  * 360);

      //  return (absEncoder.getAbsolutePosition()-absEncoder.getPositionOffset()) * 360;
        return absEncoder.getAbsolutePosition() * 360;
        //return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset())  * 360;
    }
    private double getRelativeArmPosition(){
        //return relativeEncoder.getPosition() * GEAR_RATIO  * 360;
        SmartDashboard.putNumber("rel position 1", relativeEncoder1.getPosition() * GEAR_RATIO /360);
        SmartDashboard.putNumber(" raw rel position 1", relativeEncoder1.getPosition() / 360);
        SmartDashboard.putNumber("rel position 1 real", relativeEncoder1.getPosition());
        double avgRelativeEncoder = (relativeEncoder2.getPosition() + relativeEncoder1.getPosition())  / GEAR_RATIO / 360 / 2;
        //  return (absEncoder.getAbsolutePosition()-absEncoder.getPositionOffset()) * 360;
        SmartDashboard.putNumber("avg rel position", avgRelativeEncoder);

        return avgRelativeEncoder;
        //return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset())  * 360;
    }


    public void changePidValues(){
        double p = SmartDashboard.getNumber("ARM P Gain", 0.001);
        double i = SmartDashboard.getNumber("ARM I Gain", 0);
        double d = SmartDashboard.getNumber("ARM D Gain", 0);
        double iz = SmartDashboard.getNumber("ARM I Zone", 0);
        double staticGain = SmartDashboard.getNumber("ARM Feed Forward", 0);
        double gravityG = SmartDashboard.getNumber("ARM Max Output", 0);

        double maxV = SmartDashboard.getNumber("ARM Max Velocity", 0.01);
        double velocityG = SmartDashboard.getNumber("ARM Min Velocity", 0.01);
        double maxA = SmartDashboard.getNumber("ARM Max Acceleration", 0.001);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != KP)) { motorPid.setP(p); KP = p; }
        if((i != KI)) { motorPid.setI(i); KI = i; }
        if((d != KD)) { motorPid.setD(d); KD = d; }
        if((iz != I_ZONE)) { motorPid.setIZone(iz); I_ZONE = iz; }
        
        if((maxV != MAX_VELOCITY)) {  MAX_VELOCITY = maxV; }
        if((velocityG != VELOCITY_GAIN)) {  VELOCITY_GAIN = velocityG; }
        if((maxA != MAX_ACCELERATION)) { MAX_ACCELERATION = maxA; }
        if((staticGain != STATIC_GAIN)) { STATIC_GAIN = staticGain; }
        if((gravityG != GRAVITY_GAIN)) { GRAVITY_GAIN = gravityG; }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
