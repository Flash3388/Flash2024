package frc.robot.actions;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.Swerve;

public class LimelightAutoAlign  extends ActionBase {
    private final Limelight limelight;
    private final Swerve swerve;
    private double angle2Target = 0;
    private double startingAngle = 0;
    private PidController pidController;
    private final double KP = 0.045;
    private final double KI = 0.0015;
    private final double KD = 0;
    private final double KF = 0;
    private final double PID_ERROR = 2;
    private final double PID_LIMIT = 1;



    public LimelightAutoAlign(Limelight limelight, Swerve swerve) {
        this.limelight = limelight;
        this.swerve = swerve;
        startingAngle = swerve.getHeadingDegrees();
        this.angle2Target = limelight.getXAngleToTarget() + startingAngle ;

        SmartDashboard.putNumber("KP m", KP);
        SmartDashboard.putNumber("KI m", KI);
        SmartDashboard.putNumber("KD m", KD);
        SmartDashboard.putNumber("KF m", KF);
        pidController = new PidController(RunningRobot.getControl().getClock(),
                ()-> {return SmartDashboard.getNumber("KP m", KP);},
                ()-> {return SmartDashboard.getNumber("KI m", KI);},
                ()-> {return SmartDashboard.getNumber("KD m", KD);},
                ()-> {return SmartDashboard.getNumber("KF m", KF);});
        //something
        pidController.setTolerance(PID_ERROR, Time.milliseconds(500));
        pidController.setOutputLimit(PID_LIMIT);

        //configure().setName("LimelightAutoAlign").save();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        startingAngle = swerve.getHeadingDegrees();
        SmartDashboard.putNumber("startingAngle",startingAngle);
        SmartDashboard.putNumber("X angle to target",limelight.getXAngleToTarget());

        this.angle2Target = limelight.getXAngleToTarget() + startingAngle ;
        SmartDashboard.putNumber("angle2T",angle2Target);
        pidController.reset();
    }

    @Override
    public void execute(ActionControl actionControl) {
        //distanceX = contourCenter.x - imageCenter.x;
        // axis x- to the right, axis y- down
        //actionControl.finish;
        double gyroAngle = swerve.getHeadingDegrees();

        SmartDashboard.putNumber("gyro angle", gyroAngle);
        SmartDashboard.putNumber("starting angle", startingAngle);
        SmartDashboard.putNumber("angle2Target", angle2Target);
        SmartDashboard.putNumber("graph angle2Target - current", angle2Target-gyroAngle);


        if(!ExtendedMath.constrained(gyroAngle, -PID_ERROR + angle2Target, PID_ERROR + angle2Target)) {

            // Direction rotateDirection = angle2Target < startingAngle ? Direction.BACKWARD : Direction.FORWARD; //if + then right, if - left
            double rotation = pidController.applyAsDouble(gyroAngle, angle2Target);
            // double rotation = pidController.applyAsDouble(gyroAngle, angle2Target) * swerve.MAX_SPEED;
            SmartDashboard.putNumber("rotation", -rotation);
            swerve.drive(0, 0, -rotation);
        }
        else {
            SmartDashboard.putBoolean("got to target", true);
            actionControl.finish();
        }

        // move until distanceX is as close as possible 0,
        // indicating the robot is aligned with the target


    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
