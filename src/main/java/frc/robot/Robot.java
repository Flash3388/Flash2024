package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import frc.robot.Actions.ShooterAMP;
import frc.robot.Actions.ShooterSpeaker;
import frc.robot.Actions.ReverseShooter;
import frc.robot.Subsystems.ShooterSystem;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private ShooterSystem shooter;
    private final XboxController xboxController;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);

        this.shooter = SystemFactory.createShooter();
        xboxController = getHidInterface().newXboxController(RobotMap.XBOX);
        xboxController.getButton(XboxButton.X).whileActive(new ShooterSpeaker(shooter));
        xboxController.getButton(XboxButton.A).whileActive(new ShooterAMP(shooter));
        xboxController.getButton(XboxButton.Y).whileActive(new ReverseShooter(shooter));
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
