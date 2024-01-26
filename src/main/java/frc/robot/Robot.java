package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import frc.robot.actions.TakeOut;
import frc.robot.subsystems.Intake;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Intake intake;
    private XboxController xbox;

    public Robot(FrcRobotControl robotControl)
    {
        super(robotControl);
        this.intake = SystemFactory.createIntake();
        this.xbox = getHidInterface().newXboxController(RobotMap.XBOX);
        // xbox.getButton(XboxButton.B).whileActive(new TakeOut(intake));
        // xbox.getButton(XboxButton.A).whileActive(new TakeIn(intake));

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
