package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.jmath.ExtendedMath;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private static  double KP = 0.012; // tune this
    private static  double KI = 0.00055; // tune this
    private static  double KD = 0.000001; // tune this
    private static  double I_ZONE = 15; // tune this

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
    private SimpleMotorFeedforward motorFeedForward;
    private final DutyCycleEncoder absEncoder;
    private PidController pid;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        master = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        follower = new CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless);

        follower.follow(master, true);
        master.follow(CANSparkBase.ExternalFollower.kFollowerDisabled, 0); // this is to make sure the master won't follow anyone

        motorFeedForward = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN);


        absEncoder = new DutyCycleEncoder(9);
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


         SmartDashboard.putNumber("speed master", 0);


        pid = PidController.newNamedController("drive", KP, KI, KD, 0);
        pid.setIZone(I_ZONE);
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
        pid.reset();
    }

    @Override
    public void autonomousPeriodic() {

        changePidValues();
        SmartDashboard.putNumber("follower set velocity", follower.get());
        SmartDashboard.putNumber("master set velocity", master.get());

        double setPoint = SmartDashboard.getNumber("set point A", 20); // here we get the setPoint
//
        //TrapezoidProfile.State setPointT = pid.getSetpoint(); // the TrapezoidProfile calculate its setPoints
        //double feedForward = motorFeedForward.calculate(Math.toRadians(setPointT.position), setPointT.velocity); // how we find the right feedForward

        double speed = pid.applyAsDouble(getArmPosition(), setPoint) ;
        speed = ExtendedMath.constrain(speed, -0.5, 0.5);

        SmartDashboard.putNumber("MOTOR Set Point", speed) ;
        SmartDashboard.putNumber("MOTOR Feed Forward", 0);

        master.set(speed);



    }

    private double getArmPosition(){
        return (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
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

        if (iz != I_ZONE) {
            I_ZONE = iz;
            pid.setIZone(iz);
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
