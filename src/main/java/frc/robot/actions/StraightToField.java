package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.Swerve;

public class StraightToField extends ActionBase {
    private final Limelight limelight;
    private final Swerve swerve;
    private double angleInField = 0;
    private PidController pidController;
    private final double KP = 0.08;
    private final double KI = 0.00001;
    private final double KD = 0.00;
    private final double PID_ERROR = 1;
    private final double PID_LIMIT = 1;



    public StraightToField( Limelight limelight, Swerve swerve) {
        this.limelight = limelight;
        this.swerve = swerve;
        this.angleInField = 0;
        pidController = PidController.newNamedController("rotation in automation", KP, KI, KD, 0);

        pidController.setTolerance(PID_ERROR, 0.01); //0.001

        pidController.setOutputLimit(PID_LIMIT);

        //configure().setName("LimelightAutoAlign").save();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        pidController.reset();
        angleInField = limelight.angleToForward_FieldRelative_Odometer(); //current
        SmartDashboard.putNumber("angle to forward", angleInField);
    }

    @Override
    public void execute(ActionControl actionControl) {

        double rotation = pidController.applyAsDouble(angleInField, 0 ); //using odometry
        SmartDashboard.putNumber("rotation", rotation);

        swerve.drive(0, 0, rotation, true);
        if(pidController.isInTolerance())
            actionControl.finish();
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}

