package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.actions.Action;
import com.flash3388.flashlib.scheduling.actions.Actions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.*;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.ShooterSystem;
import frc.robot.subSystems.Swerve;
import com.flash3388.flashlib.robot.control.PidController;
import com.jmath.ExtendedMath;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.ArmController;
import frc.robot.subSystems.Arm;

import javax.swing.*;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private Intake intake;
    private ShooterSystem shooter;
    private Limelight limelight;

    private final XboxController xbox;


   private Arm arm;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        this.arm = SystemFactory.createArm();
        swerve = SystemFactory.createSwerveSystem();
        intake = SystemFactory.createIntake();
        shooter = SystemFactory.createShooter();
        xbox = getHidInterface().newXboxController(RobotMap.XBOX);
        limelight = new Limelight(swerve);

        xbox.getButton(XboxButton.X).whenActive(new LimelightAutoAlign(limelight,swerve));
        xbox.getDpad().down().whenActive(Actions.instant(() -> arm.setPositioningNotControlled()));
        xbox.getDpad().up().whileActive(new ShooterSpeaker(shooter, intake));
        xbox.getButton(XboxButton.A).whileActive(new ShooterAMP(shooter, intake));
        xbox.getButton(XboxButton.Y).whileActive(new ReverseShooter(shooter));
        xbox.getButton(XboxButton.Y).whileActive(new TakeOut(intake));
        xbox.getButton(XboxButton.B).whenActive(new TakeIn(intake));
        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox));
        arm.setDefaultAction(new ArmController(arm));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {


    }

    @Override
    public void autonomousInit() {


    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {
        arm.resetPID();
        shooter.resetI();
    }

    @Override
    public void testPeriodic() {
        double angle2T = limelight.getXAngleToTarget();
        SmartDashboard.putNumber("angle2T",angle2T); //check if the value is correct-it may be the wrong one, so switch

        double distance = limelight.getDistanceToTarget();
        SmartDashboard.putNumber("distance",distance); //it may not work 100% accuratly, i need to tune it when i'm in the room


        double actud= Math.cos((limelight.getYAngleToTarget()+25)/distance);
        SmartDashboard.putNumber("actual distance",actud); //it may not work 100% accuratly, i need to tune it when i'm in the room

        double cameraHeight = 0.485;
        double actualDis = Math.sqrt(Math.pow(distance,2) - Math.pow(limelight.getTargetHeight() - cameraHeight,2));
        SmartDashboard.putNumber("hopefully real distance",actualDis);

        double setPoint = SmartDashboard.getNumber("set point A", Arm.FLOOR_ANGLE);
        arm.setSetPointAngle(setPoint);
        SmartDashboard.putBoolean("see target",limelight.isThereTarget());


       /* SmartDashboard.putNumber("rotation",swerve.getPose2D().getRotation().getRadians());
        SmartDashboard.putNumber("xTrans",swerve.getPose2D().getTranslation().getX());
        SmartDashboard.putNumber("yTrans",swerve.getPose2D().getTranslation().getY());*/

        shooter.changePidValues();
      //  shooter.setVelocity(SmartDashboard.getNumber("set point velocity", 0));
    }

    @Override
    public void robotPeriodic() {
       arm.print();
        limelight.updateRobotPositionByAprilTag();
        swerve.updateOdometer();
        shooter.print();
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
