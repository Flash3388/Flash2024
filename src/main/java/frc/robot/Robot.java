package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.LimelightAutoAlign;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.Swerve;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private ShooterSystem shooter;
    private final XboxController xbox;
    private Limelight limelight;
    private DigitalInput in = new DigitalInput(6);

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        swerve = SystemFactory.createSwerveSystem();
        xbox = getHidInterface().newXboxController(RobotMap.XBOX);
        shooter = SystemFactory.createShooter();
        limelight = new Limelight(swerve);


        xbox.getButton(XboxButton.X).whileActive(new LimelightAutoAlign(limelight,swerve));
   //     xbox.getButton(XboxButton.Y).whileActive(new ReverseShooter(shooter));
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
        SmartDashboard.putBoolean("SeeTarget",false);

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

    }

    @Override
    public void testPeriodic() {

        double angle2T = limelight.getXAngleToTarget();
        SmartDashboard.putNumber("angle2T",angle2T); //check if the value is correct-it may be the wrong one, so switch
        // to cameraPoseTargetSpace[4]

        double distance = limelight.getDistanceToTarget();
        SmartDashboard.putNumber("distance",distance); //it may not work 100% accuratly, i need to tune it when i'm in the room
    }

    @Override
    public void robotPeriodic() {
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
