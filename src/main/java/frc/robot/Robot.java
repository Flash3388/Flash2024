package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.actions.ActionGroup;
import com.jmath.ExtendedMath;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.ArmController;

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
    private static  double KP = 0.01; // tune this
    private static  double KI = 0.0014; // tune this
    private static  double KD = 0; // tune this
    private static  double I_ZONE = 0; // tune this

    // for trapezoid profile
    private static  double MAX_VELOCITY = 1; // meters per second
    private static  double MAX_ACCELERATION = 1; // meters per second squared

    // for feed forward
    // can use this to help tune

    private static  double STATIC_GAIN = 0; // volts // tune this
    private static  double GRAVITY_GAIN = 0; // volts // tune this
    private static  double VELOCITY_GAIN = 0; // volts * seconds / radians // tune this

    private static final double LOOP_TIME_SECONDS = 0.02; // this is a global constant for the robot code
    private static final double GEAR_RATIO = 1.0/70.0  ; // driver / driven
    private static final int PID_SLOT = 0; // default slot to use

    private final CANSparkMax motor;
    private final CANSparkMax follower;
    private ArmFeedforward motorFeedForward;
    private final DutyCycleEncoder absEncoder;

    private ProfiledPIDController pid;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        follower = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);
        follower.follow(motor, true);

        motorFeedForward = new ArmFeedforward(STATIC_GAIN, GRAVITY_GAIN, VELOCITY_GAIN);



        absEncoder = new DutyCycleEncoder(9);
      //  absEncoder.setPositionOffset(81.79668 / 360);
        absEncoder.setPositionOffset(81.79668 / 360);

     // I have found out that the method "absEncoder.getAbsolutePosition()" returns 0 in the constructor. this must be the problem.
          SmartDashboard.putNumber("Abs positon start", (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) *360);

         SmartDashboard.putNumber("ARM P Gain", KP);
         SmartDashboard.putNumber("ARM I Gain", KI);
         SmartDashboard.putNumber("ARM D Gain", KD);
         SmartDashboard.putNumber("ARM IZ Zone", I_ZONE);
         SmartDashboard.putNumber("ARM Static Gain", 0);
         SmartDashboard.putNumber("ARM Gravity", 0);

         SmartDashboard.putNumber("ARM Max Velocity", 0.01);
         SmartDashboard.putNumber("ARM Velocity Gain", 0.0);
         SmartDashboard.putNumber("ARM Max Acceleration", 0.071);


        pid = new ProfiledPIDController(KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
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
    }

    @Override
    public void autonomousPeriodic() {
        changePidValues();
        /*motorPid.setReference(
                positionForMotor,
                CANSparkBase.ControlType.kPosition,
                PID_SLOT,
                feedForward,
                SparkPIDController.ArbFFUnits.kVoltage);*/

       // motorPid.setReference(9 / GEAR_RATIO / 360.0, CANSparkBase.ControlType.kPosition);

        double speed = pid.calculate(getArmPosition(), 10);
        speed = ExtendedMath.constrain(speed, -0.5, 0.5);
        SmartDashboard.putNumber("MOTOR Set Point", speed);
        // SmartDashboard.putNumber("MOTOR Feed Forward", feedForward);
        motor.set(speed);

       // SmartDashboard.putNumber("OFFSET", absEncoder.getAbsolutePosition() * 360);
    }

    private double getArmPosition(){
        // return relativeEncoder.getPosition() * GEAR_RATIO  * 360;
      //  SmartDashboard.putNumber("rel position", relativeEncoder1.getPosition() * GEAR_RATIO  * 360);

      //  return (absEncoder.getAbsolutePosition()-absEncoder.getPositionOffset()) * 360;
        return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
        //return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset())  * 360;
    }


    public void changePidValues(){
        double p = SmartDashboard.getNumber("ARM P Gain", 0.3);
        double i = SmartDashboard.getNumber("ARM I Gain", 0);
        double d = SmartDashboard.getNumber("ARM D Gain", 0);
        double iz = SmartDashboard.getNumber("ARM IZ Zone", 0);
        double staticGain = SmartDashboard.getNumber("ARM Static Gain", 0);
        double gravityG = SmartDashboard.getNumber("ARM Gravity", 1);

        double maxV = SmartDashboard.getNumber("ARM Max Velocity", 0.01);
        double velocityG = SmartDashboard.getNumber("ARM Velocity Gain", 0.01);
        double maxA = SmartDashboard.getNumber("ARM Max Acceleration", .01);

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

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("ABS Arm position", getArmPosition());
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
