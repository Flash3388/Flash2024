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
    private final double KP = 0.08;
    private final double KI = 0.00001;
    private final double KD = 0.00;
    private final double KF = 0;
    private final double PID_ERROR = 0.5;
    private final double PID_LIMIT = 1;
    private boolean continuous;
    private boolean withXbox;
    private double signum = 1;



    public LimelightAutoAlignWithDrive(XboxController xbox_driver, Limelight limelight, Swerve swerve, Arm arm,
                                       boolean continuous,
                                       boolean withXbox) {
        this.xbox_driver = xbox_driver;
        this.limelight = limelight;
        this.swerve = swerve;
        this.arm = arm;
        this.angle2Target = 0;
        this.continuous = continuous;
        this.withXbox = withXbox;

        pidController = PidController.newNamedController("rotation", KP, KI, KD, 0);

        pidController.setTolerance(PID_ERROR, 0.001);
        pidController.setOutputLimit(PID_LIMIT);

        //configure().setName("LimelightAutoAlign").save();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        pidController.reset();
        signum = 1;
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (!allianceOptional.isEmpty()) {
            DriverStation.Alliance alliance = allianceOptional.get();
            if(alliance == DriverStation.Alliance.Blue)
                signum = -1;
        }
    }

    @Override
    public void execute(ActionControl actionControl) {
        double gyroAngle = swerve.getHeadingDegrees();

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
            driveX = signum * xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
            driveX = Math.abs(driveX) > 0.2 ? driveX * Swerve.MAX_SPEED/2 : 0;
            driveY = signum * xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
            driveY = Math.abs(driveY) > 0.2 ? driveY * Swerve.MAX_SPEED/2: 0;
        }

       double rotation = pidController.applyAsDouble(gyroAngle, angle2Target); //using odometry


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
