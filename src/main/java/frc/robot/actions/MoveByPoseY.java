package frc.robot.actions;

import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subSystems.Swerve;

public class MoveByPoseY extends ActionBase {
    private Swerve swerve;
    private double placeY;
    private double currentY_Place;
    private double setPoint;
    private PidController pid;
    private static  double KP_DISTANCE = 0.06;
    private static  double KI_DISTANCE = 0.001;
    private static  double KD_DISTANCE = 0;
    private static final double ERROR = 0.02;

    public MoveByPoseY(Swerve swerve, double placeY){
        this.swerve = swerve;
        this.placeY = placeY;
        this.pid = PidController.newNamedController("moveDistanceByPose", KP_DISTANCE, KI_DISTANCE, KD_DISTANCE, 0);
        pid.setTolerance(ERROR, Double.POSITIVE_INFINITY);

        pid.setOutputLimit(1);
        requires(swerve);
    }
    @Override
    public void initialize(ActionControl control) {
        currentY_Place = swerve.getRobotPose().getY();
        //setPoint = currentY_Place + placeY;
        pid.reset();
    }

    @Override
    public void execute(ActionControl control) {
        currentY_Place = swerve.getRobotPose().getY();
        double speed = pid.applyAsDouble(currentY_Place, placeY) * Swerve.MAX_SPEED;

        swerve.drive(speed, 0, 0);
        if(pid.isInTolerance()){
            control.finish();
        }
    }

    @Override
    public void end(FinishReason reason) {
        swerve.drive(0, 0, 0);
    }
}
