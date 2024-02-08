package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.actions.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.Swerve;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private Intake intake;
    private ShooterSystem shooter;
    private Limelight limelight;

    private final XboxController xbox;
    private DigitalInput in = new DigitalInput(6);


    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        this.intake = SystemFactory.createIntake();
        this.shooter = SystemFactory.createShooter();
        xbox = getHidInterface().newXboxController(RobotMap.XBOX);

        xbox.getButton(XboxButton.X).whenActive(new LimelightAutoAlign(limelight,swerve));

       // xboxController.getButton(XboxButton.X).whileActive(new ShooterSpeaker(shooter));
        xbox.getButton(XboxButton.A).whileActive(new ShooterAMP(shooter));
   //     xboxController.getButton(XboxButton.Y).whileActive(new ReverseShooter(shooter));
        xbox.getButton(XboxButton.Y).whileActive(new TakeOut(intake));
        xbox.getButton(XboxButton.B).whenActive(new TakeIn(intake));

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
