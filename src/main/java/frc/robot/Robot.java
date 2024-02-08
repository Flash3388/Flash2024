package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
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

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private Intake intake;
    private ShooterSystem shooter;
    private Limelight limelight;

    private final XboxController xbox;
    private DigitalInput in = new DigitalInput(6);


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

        xbox.getDpad().up().whileActive(new ShooterSpeaker(shooter, intake));
        xbox.getButton(XboxButton.A).whileActive(new ShooterAMP(shooter, intake));
        xbox.getButton(XboxButton.Y).whileActive(new ReverseShooter(shooter));
        xbox.getButton(XboxButton.Y).whileActive(new TakeOut(intake));
        xbox.getButton(XboxButton.B).whenActive(new TakeIn(intake));
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
        boolean on = in.get(); //if the sensor senses a note
        SmartDashboard.putBoolean("isNoteIn",on);
        double driveY = -xbox.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
        double driveX = -xbox.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
        double rotation = -xbox.getAxis(XboxAxis.RightStickX).getAsDouble();

        driveY = Math.abs(driveY) > 0.2 ? driveY * Swerve.MAX_SPEED    : 0;
        driveX = Math.abs(driveX) > 0.2 ? driveX  * Swerve.MAX_SPEED : 0;
        rotation = Math.abs(rotation) > 0.4 ? rotation * Swerve.MAX_SPEED : 0;

        boolean isFiledRelative = SmartDashboard.getBoolean("Is Field Relative?", false);
        // this.swerve.drive(driveY /3 ,driveX/3 ,rotation/3, true);
        this.swerve.drive(driveY/3,driveX/3,rotation/3);
        SmartDashboard.putNumber("rotation",swerve.getPose2D().getRotation().getRadians());
        SmartDashboard.putNumber("xTrans",swerve.getPose2D().getTranslation().getX());
        SmartDashboard.putNumber("yTrans",swerve.getPose2D().getTranslation().getY());
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
    }

    @Override
    public void testPeriodic() {
        double angle2T = limelight.getXAngleToTarget();
        SmartDashboard.putNumber("angle2T",angle2T); //check if the value is correct-it may be the wrong one, so switch
        // to cameraPoseTargetSpace[4]

        double distance = limelight.getDistanceToTarget();
        SmartDashboard.putNumber("distance",distance); //it may not work 100% accuratly, i need to tune it when i'm in the room

        double setPoint = SmartDashboard.getNumber("set point A", 20);
        arm.setSetPointAngle(setPoint);
    }

    @Override
    public void robotPeriodic() {
       arm.print();
        limelight.updateRobotPositionByAprilTag();
        swerve.updateOdometer();
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
