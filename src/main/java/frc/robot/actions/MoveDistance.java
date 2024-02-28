package frc.robot.actions;

import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.jmath.ExtendedMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Swerve;

public class MoveDistance extends ActionBase {

    private Swerve swerve;
    private PidController pid;
    private double distance;
    private double setPoint;

    private static  double KP_DISTANCE = 0.06; // tune this
    private static  double KI_DISTANCE = 0.001; // tune this
    private static  double KD_DISTANCE = 0; // tune this
    //private static  double I_ZONE = 15; // tune this

    private static final double ERROR = 0.01;

    private boolean isFieldRelative;

    public MoveDistance(Swerve swerve){
        this(swerve, 0, false);
    }

    public MoveDistance(Swerve swerve, double distance, boolean isFieldRelative){
        this.swerve = swerve;
        this.distance = distance;
        this.isFieldRelative = false;
        this.pid = PidController.newNamedController("moveDistance", KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, 0);
        pid.setTolerance(ERROR, Double.POSITIVE_INFINITY);

        pid.setOutputLimit(1);
        requires(swerve);
    }

    public void setDistance(double distance){
        this.distance = distance;
    }

    @Override
    public void initialize(ActionControl control) {
        SmartDashboard.putNumber("bla-1", swerve.getDistancePassedMeters());
        swerve.resetDistancePassed();
        swerve.resetWheels();
        setPoint = distance;
        SmartDashboard.putNumber("bla", swerve.getDistancePassedMeters());
        SmartDashboard.putNumber("Set Point Number", setPoint);
        pid.reset();
    }

    @Override
    public void execute(ActionControl control) {
        double distancePassed = -swerve.getDistancePassedMeters();;

        if(ExtendedMath.constrained(swerve.getFLHeading(), 170, 190))
            distancePassed = -distancePassed;

        double speed = pid.applyAsDouble(distancePassed, setPoint) * Swerve.MAX_SPEED *1.5;


        swerve.drive(speed, 0, 0, isFieldRelative);
        SmartDashboard.putNumber("pid's speed", speed);
        SmartDashboard.putNumber("Drive Distance Action", distancePassed);

        if(pid.isInTolerance()){
            control.finish();
        }
    }

    @Override
    public void end(FinishReason reason) {
        swerve.drive(0, 0, 0);

    }
}
