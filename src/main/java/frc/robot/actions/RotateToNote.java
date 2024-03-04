package frc.robot.actions;

import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subSystems.RaspberryPi;
import frc.robot.subSystems.Swerve;

public class RotateToNote extends ActionBase {
    private RaspberryPi camera;
    private Swerve swerve;
    private PidController pidController;
    private double setPoint = 0;
    private final double KP = 0.08; //0.08
    private final double KI = 0.00002; // 0.00001  0.00002
    private final double KD = 0.00;
    private final double KF = 0;
    private final double I_ZONE = 5;
    private final double PID_ERROR = 1;
    private final double PID_LIMIT = 1;

    public RotateToNote(RaspberryPi raspberryPi, Swerve swerve){
        this.camera = raspberryPi;
        this.swerve = swerve;

        pidController = PidController.newNamedController("AutoAlignWithDrive.rotation", KP, KI, KD, 0);

        pidController.setTolerance(PID_ERROR, 0.1); //0.001 0.01
        pidController.setOutputLimit(PID_LIMIT);
        pidController.setIZone(I_ZONE);

        requires(swerve);
    }


    @Override
    public void initialize(ActionControl control) {
    }

    @Override
    public void execute(ActionControl control) {
        double rotation = pidController.applyAsDouble(camera.getXAngleToTarget(), setPoint);
        swerve.drive(0, 0, rotation);

        if(pidController.isInTolerance())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
