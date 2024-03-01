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
    private PidController pidController;
    private final double KP = 0.08;
    private final double KI = 0.00002; // 0.00001
    private final double KD = 0.00;
    private final double KF = 0;
    private final double PID_ERROR = 0.7;
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

        pidController = PidController.newNamedController("rotation", KP, KI, KD, 0);

        pidController.setTolerance(PID_ERROR, 0.01); //0.001
        pidController.setOutputLimit(PID_LIMIT);

        this.clock = RunningRobot.getControl().getClock();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        pidController.reset();
        time = Time.INVALID;

    }

    @Override
    public void execute(ActionControl actionControl) {
        angle2Target = limelight.getXAngleToTarget_Amp();
        distance2Target = limelight.getXDistanceToTarget_Amp();
        double driveX = 1;
        double driveY = 1;
        if (Math.abs(distance2Target) <= 0.03)
            driveX = 0;
        else
            driveX = distance2Target > 0 ? -0.5 : 0.5;
        driveY = -xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble();
        driveY = Math.abs(driveY) > 0.2 ? driveY * Swerve.MAX_SPEED : 0;
        double rotation = pidController.applyAsDouble(angle2Target, 0);

        if(driveX != 0)
            swerve.drive(0, driveX, 0, false);
        else if(rotation != 0) // in this the driveX == 0
            swerve.drive(0, 0, rotation, false);
        else //rotaion == driveX == 0
            swerve.drive(driveY, 0, 0, false);

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
    }
}

