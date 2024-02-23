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

public class LimelightAutoAlignWithDrive extends ActionBase {
    private XboxController xbox_driver;
    private final Limelight limelight;
    private final Swerve swerve;
    private final Arm arm;
    private double angle2Target = 0;
    private PidController pidController;
    private final double KP = 0.09;
    private final double KI = 0.00001;
    private final double KD = 0.00;
    private final double KF = 0;
    private final double PID_ERROR = 0.5;
    private final double PID_LIMIT = 1;
    private boolean continuous;
    private boolean withXbox;



    public LimelightAutoAlignWithDrive(XboxController xbox_driver, Limelight limelight, Swerve swerve, Arm arm,
                                       boolean continuous,
                                       boolean withXbox) {
        this.xbox_driver = xbox_driver;
        this.limelight = limelight;
        this.swerve = swerve;
        this.arm = arm;
      //  this.angle2Target = limelight.getXAngleToTarget() + startingAngle ;
        this.angle2Target = 0;
        this.continuous = continuous;
        this.withXbox = withXbox;

        pidController = PidController.newNamedController("rotation", KP, KI, KD, 0);
        //something
        pidController.setTolerance(PID_ERROR, 0.001);
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
        //distanceX = contourCenter.x - imageCenter.x;
        // axis x- to the right, axis y- down
        //actionControl.finish;
        double gyroAngle = swerve.getHeadingDegrees();
       // double currentDistance = swerve.getDistancePassedMeters();

        if(arm.isSetToAMP())
            angle2Target = limelight.getXAngleToTarget_Amp() + gyroAngle;
        else
            this.angle2Target = limelight.getXAngleToTarget_Speaker() + gyroAngle;

        SmartDashboard.putNumber("VisionAlign GyroAngle", gyroAngle);
        SmartDashboard.putBoolean("VisionAlign SetToAMP", arm.isSetToAMP());
        SmartDashboard.putNumber("VisionAlign AngleToTarget", angle2Target);

        double rotation2 = 0;
        double driveX = 0;
        double driveY = 0;
        if (withXbox) {
            rotation2 = -xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble();
            rotation2 = Math.abs(rotation2) > 0.2 ? rotation2 : 0;
            driveX = -xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
            driveX = Math.abs(driveX) > 0.2 ? driveX : 0;
            driveY = -xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
            driveY = Math.abs(driveY) > 0.2 ? driveY : 0;
        }
       // double rotation = pidController.applyAsDouble(gyroAngle, angle2Target);

       double rotation = pidController.applyAsDouble(gyroAngle, angle2Target); //using odometry


      /*  if(limelight.isThereTarget()){ //only if sees target + not good enough so that the odometer will be good -> use joystick
            if(limelight.getAvgDistance() > 2.8 ) rotation = rotation2; //optionally create a pipeline that can't see targets
            //so that the odometer will work- but if so, when do i turn it off and return to original
        }*/




       /* double rotation = limelight.isThereTarget() ?
                pidController.applyAsDouble(gyroAngle, angle2Target):
                rotation2;
         */

        if (!continuous && pidController.isInTolerance())  {
            actionControl.finish();
            return;
        }

        //     double xDrive = pidController.applyAsDouble(gyroAngle, angle2Target);
        // double rotation = pidController.applyAsDouble(gyroAngle, angle2Target) * swerve.MAX_SPEED;

        SmartDashboard.putNumber("rotation", rotation);
        swerve.drive(driveY, driveX, rotation);

        // move until distanceX is as close as possible 0,
        // indicating the robot is aligned with the target
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
