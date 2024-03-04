package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.robot.control.PidController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.Swerve;

import java.util.Optional;

public class LimelightAutoAlignWithDrive extends ActionBase {
    private XboxController xbox_driver;
    private final Limelight limelight;
    private final Swerve swerve;
    private final Arm arm;
    private double angle2Target = 0;
    private PidController pidController;
    private final double KP = 0.08; //0.08
    private final double KI = 0.00002; // 0.00001  0.00002
    private final double KD = 0.00;
    private final double KF = 0;
    private final double PID_ERROR = 1; //0.7
    private final double PID_LIMIT = 1;
    private boolean continuous;
    private boolean withXbox;
    private double signum = 1;

    private Intake intake;

    public LimelightAutoAlignWithDrive(XboxController xbox_driver, Limelight limelight, Swerve swerve, Arm arm, Intake intake,
                                       boolean continuous,
                                       boolean withXbox) {
        this.xbox_driver = xbox_driver;
        this.limelight = limelight;
        this.swerve = swerve;
        this.arm = arm;
        this.angle2Target = 0;
        this.continuous = continuous;
        this.withXbox = withXbox;
        this.intake = intake;

        pidController = PidController.newNamedController("AutoAlignWithDrive.rotation", KP, KI, KD, 0);

        pidController.setTolerance(PID_ERROR, 0.1); //0.001 0.01
        pidController.setOutputLimit(PID_LIMIT);

        //configure().setName("LimelightAutoAlign").save();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        pidController.reset();


    }

    @Override
    public void execute(ActionControl actionControl) {
        double gyroAngle = swerve.getHeadingDegrees();

        if(!intake.isIN() && !DriverStation.isAutonomous())
            actionControl.finish();

        if(arm.isSetToAMP())
            angle2Target = limelight.getXAngleToTarget_Amp() + gyroAngle;
        else
            this.angle2Target = limelight.getXAngleToTarget_Speaker() + gyroAngle;

        SmartDashboard.putNumber("VisionAlign GyroAngle", gyroAngle);
        SmartDashboard.putBoolean("VisionAlign SetToAMP", arm.isSetToAMP());
        SmartDashboard.putNumber("VisionAlign AngleToTarget", angle2Target);

        double driveX = 0;
        double driveY = 0;
        if (withXbox) {
            driveX = Swerve.SIGNUM * xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
            driveX = Math.abs(driveX) > 0.2 ? driveX * driveX * Math.signum(driveX) * Swerve.MAX_SPEED : 0;
            driveY = Swerve.SIGNUM * xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
            driveY = Math.abs(driveY) > 0.2 ? driveY * driveY * Math.signum(driveY) * Swerve.MAX_SPEED: 0;
        }

       double rotation = pidController.applyAsDouble(gyroAngle, angle2Target); //using odometry
      // rotation = Math.abs(rotation) > 0.2 ? rotation : 0;


        if (!continuous && pidController.isInTolerance())  {
            actionControl.finish();
            return;
        }

        SmartDashboard.putNumber("rotation", rotation);
        swerve.drive(driveY, driveX, rotation, true);
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
