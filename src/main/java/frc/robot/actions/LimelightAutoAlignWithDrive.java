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
    private double startingAngle = 0;//
    private PidController pidController;
    private final double KP = 0.09;
    private final double KI = 0.00001;
    private final double KD = 0.00;
    private final double KF = 0;
    private final double PID_ERROR = 2;
    private final double PID_LIMIT = 1;



    public LimelightAutoAlignWithDrive(XboxController xbox_driver, Limelight limelight, Swerve swerve, Arm arm) {
        this.xbox_driver = xbox_driver;
        this.limelight = limelight;
        this.swerve = swerve;
        this.arm = arm;
        startingAngle = swerve.getHeadingDegrees();
      //  this.angle2Target = limelight.getXAngleToTarget() + startingAngle ;
        this.angle2Target = 0;

        SmartDashboard.putNumber("KP m", KP);
        SmartDashboard.putNumber("KI m", KI);
        SmartDashboard.putNumber("KD m", KD);
        SmartDashboard.putNumber("KF m", KF);
        pidController = PidController.newNamedController("rotation", KP, KI, KD, 0);
        //something
        pidController.setTolerance(PID_ERROR, Double.POSITIVE_INFINITY);
        pidController.setOutputLimit(PID_LIMIT);

        //configure().setName("LimelightAutoAlign").save();

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {
        startingAngle = swerve.getHeadingDegrees();
        SmartDashboard.putNumber("startingAngle",startingAngle);
        //how do i know to which state i wanna be in

        SmartDashboard.putNumber("X angle to target",angle2Target -startingAngle);
        SmartDashboard.putNumber("angle2T",angle2Target);
        pidController.reset();
    }

    @Override
    public void execute(ActionControl actionControl) {
        //distanceX = contourCenter.x - imageCenter.x;
        // axis x- to the right, axis y- down
        //actionControl.finish;
        double gyroAngle = swerve.getHeadingDegrees();
       // double currentDistance = swerve.getDistancePassedMeters();

        SmartDashboard.putBoolean("is set AMP", arm.isSetToAMP());
        if(arm.isSetToAMP()){
            angle2Target = limelight.getXAngleToTarget_Amp() + gyroAngle;
            //make the wheels be at 90 degrees
            SmartDashboard.putNumber("X angle to target",angle2Target);
        }
        else
            this.angle2Target = limelight.getXAngleToTarget_Speaker() + gyroAngle;


        SmartDashboard.putNumber("gyro angle", gyroAngle);
        SmartDashboard.putNumber("angle2Target", angle2Target);
        SmartDashboard.putNumber("angle2Target new new", angle2Target);
       // SmartDashboard.putNumber("graph angle2Target - current", angle2Target-gyroAngle);

        double rotation2 = -xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble();
        rotation2 = Math.abs(rotation2) > 0.2 ? rotation2 : 0;
        double driveX = -xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
        double driveY = -xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
        double rotation = pidController.applyAsDouble(gyroAngle, angle2Target);


        /*
        double rotation = limelight.isThereTarget() ?
                pidController.applyAsDouble(gyroAngle, angle2Target):
                rotation2;
         */
        driveY = Math.abs(driveY) > 0.2 ? driveY : 0;
        driveX = Math.abs(driveX) > 0.2 ? driveX : 0;
        rotation = Math.abs(rotation) > 0.2 ? rotation : 0;

        //     double xDrive = pidController.applyAsDouble(gyroAngle, angle2Target);
        // double rotation = pidController.applyAsDouble(gyroAngle, angle2Target) * swerve.MAX_SPEED;
        SmartDashboard.putNumber("rotation", -rotation);
        swerve.drive(driveY, driveX, rotation);
        // move until distanceX is as close as possible 0,
        // indicating the robot is aligned with the target


    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
