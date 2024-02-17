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


    public MoveDistance(Swerve swerve){
        this(swerve, 0);
    }

    public MoveDistance(Swerve swerve, double distance){
        this.swerve = swerve;
        this.distance = distance;
        this.pid = PidController.newNamedController("moveDistance", KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, 0);
        pid.setOutputLimit(1);
        requires(swerve);
    }

    public void setDistance(double distance){
        this.distance = distance;
    }

    @Override
    public void initialize(ActionControl control) {
        setPoint = swerve.getDistancePassedMeters() + distance;
        SmartDashboard.putNumber("Set Point Number", setPoint);
        pid.reset();
    }

    @Override
    public void execute(ActionControl control) {
        double speed = pid.applyAsDouble(swerve.getDistancePassedMeters(), setPoint) * Swerve.MAX_SPEED;

        swerve.drive(speed, 0, 0);
        SmartDashboard.putNumber("pid's speed", speed);


        if(ExtendedMath.constrained(swerve.getDistancePassedMeters(), setPoint - ERROR, setPoint + ERROR)){
            control.finish();
        }
    }

    @Override
    public void end(FinishReason reason) {
        swerve.drive(0, 0, 0);

    }
}
