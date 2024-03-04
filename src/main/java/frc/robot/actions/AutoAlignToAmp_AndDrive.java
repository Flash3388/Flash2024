package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.Swerve;

public class AutoAlignToAmp_AndDrive extends ActionBase {

    private XboxController xbox_driver;
    private final Limelight limelight;
    private final Swerve swerve;
    private final Intake intake;
    private PidController pidController_rotation;
    private PidController pidController_drive;
    private final double KP_ROTAION = 0.08;
    private final double KI_ROTATION = 0.00002; // 0.00001
    private final double KD_ROTATION = 0.00;
    private final double KF_ROTATION = 0;
    private final double KP_DRIVE = 0.37;
    private final double KI_DRIVE = 0.00001; // 0.00001
    private final double KD_DRIVE = 0.00;
    private final double KF_DRIVE = 0;

    private final double PID_ERROR_ROTATION = 0.7;
    private final double PID_ERROR_DRIVE = 0.05;
    private final double PID_LIMIT = 1;
    private double angle2Target;
    private double distance2Target;
    private static final double DELAY_BEFORE_FINISH_IN_SECONDS = 2;
    private Clock clock;
    private Time time;


    public AutoAlignToAmp_AndDrive(XboxController xbox_driver, Limelight limelight, Swerve swerve, Intake intake) {
        this.xbox_driver = xbox_driver;
        this.limelight = limelight;
        this.swerve = swerve;
        this.intake = intake;
        this.angle2Target = 0;
        this.distance2Target = 0;

        pidController_rotation = PidController.newNamedController("AutoAlignToAmp.Rotation", KP_ROTAION, KI_ROTATION, KD_ROTATION, 0);
        pidController_rotation.setTolerance(PID_ERROR_ROTATION, Double.POSITIVE_INFINITY); //0.001
        pidController_rotation.setOutputLimit(PID_LIMIT);

        pidController_drive = PidController.newNamedController("AutoAlignToAmp.Drive", KP_DRIVE, KI_DRIVE, KD_DRIVE, 0);
        pidController_drive.setTolerance(PID_ERROR_DRIVE, Double.POSITIVE_INFINITY); //0.001
        pidController_drive.setOutputLimit(PID_LIMIT);

        this.clock = RunningRobot.getControl().getClock();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        pidController_rotation.reset();
        pidController_drive.reset();
        time = Time.INVALID;
        //Limelight.KEEP_UPDATING_ODOMETER = false;
    }

    @Override
    public void execute(ActionControl actionControl) {
        angle2Target = limelight.getXAngleToTarget_Amp();
        distance2Target = limelight.getXDistanceToTarget_Amp();
        double driveX = 1;
        double driveY = 1;

        driveY = -xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble();
        driveY = Math.abs(driveY) > 0.05 ? driveY * driveY * Math.signum(driveY) * Swerve.MAX_SPEED : 0; //0.2

        driveX = pidController_drive.applyAsDouble(distance2Target, 0) * Swerve.MAX_SPEED;
        if(pidController_drive.isInTolerance())
            driveX = 0;

        double rotation = pidController_rotation.applyAsDouble(angle2Target, 0);
        if(pidController_rotation.isInTolerance())
            rotation = 0;

        SmartDashboard.putNumber("rotation amp speed", rotation);
        SmartDashboard.putNumber("drive X amp speed", driveX);
        SmartDashboard.putNumber("drive Y amp speed", driveY);

        swerve.drive(driveY, driveX, -rotation, false);


       /* if(driveX != 0 || rotation != 0)
            swerve.drive(driveY, driveX, -rotation, false);
        else
            swerve.drive(driveY, 0, 0, false);
        */

       /*
        if(driveX != 0)
            swerve.drive(0, driveX, rotation, false);
        else if(rotation != 0) // in this the driveX == 0
            swerve.drive(0, 0, rotation, false);
        else //rotaion == driveX == 0
            swerve.drive(driveY, 0, 0, false);
        */

        if (!intake.isIN()) {
            if (time.isValid()) {
                if (time.before(clock.currentTime()))
                    actionControl.finish();
            } else
                time = clock.currentTime().add(Time.seconds(DELAY_BEFORE_FINISH_IN_SECONDS));
        } else
            time = Time.INVALID;
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
        //Limelight.KEEP_UPDATING_ODOMETER = true;
    }
}

