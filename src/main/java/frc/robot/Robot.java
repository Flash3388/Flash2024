package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.DriveWithXbox;
import frc.robot.subSystems.Swerve;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {

    private Swerve swerve;
    private XboxController xbox;
    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        swerve = SystemFactory.createSwerveSystem();
        this.xbox = getHidInterface().newXboxController(RobotMap.XBOX);

        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putBoolean("Is Field Relative?", true);
        //swerve.moveWheelsForward();
    }

    @Override
    public void teleopPeriodic() {
        swerve.print();
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

    }

    @Override
    public void robotPeriodic() {

    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
