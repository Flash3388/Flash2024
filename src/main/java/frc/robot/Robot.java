package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.RoboRio;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.ArmController;
import frc.robot.subSystems.Arm;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {

    private Arm arm;
    private XboxController xbox;
    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        arm = SystemFactory.createArm();
        xbox = getHidInterface().newXboxController(RoboRio.newHidChannel(0));

        //xbox.getButton(XboxButton.B).whenActive(new AutoAim(arm, vision));

        /* possible?
        xbox.getButton(XboxButton.A).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Amp));
        xbox.getButton(XboxButton.B).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Speaker));
        xbox.getButton(XboxButton.X).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Floor));
        */

        //arm.setDefaultAction(new ArmController(arm));
        arm.setPositioningNotControlled();
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
        SmartDashboard.putNumber("setpointy", 0);
        arm.pidReset();
    }

    @Override
    public void testPeriodic() {
        double d = SmartDashboard.getNumber("setpointy", 0);
        arm.moveToAngle(d);
    }

    @Override
    public void robotPeriodic() {
        arm.print();
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
