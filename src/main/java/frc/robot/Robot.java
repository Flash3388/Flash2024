package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.RoboRio;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;

import frc.robot.actions.*;
import frc.robot.subSystems.Arm;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {

    private Arm arm;
    private XboxController xbox;
    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        arm = SystemFactory.createArm();
        xbox = getHidInterface().newXboxController(RoboRio.newHidChannel(0));

        xbox.getButton(XboxButton.A).whenActive(new MoveToFloor(arm));
        xbox.getButton(XboxButton.Y).whenActive(new MoveToSpeakerFixed(arm));
        xbox.getButton(XboxButton.X).whenActive(new MoveToAmp(arm));
        //xbox.getButton(XboxButton.B).whenActive(new AutoAim(arm, vision));

        /* possible?
        xbox.getButton(XboxButton.A).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Amp));
        xbox.getButton(XboxButton.B).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Speaker));
        xbox.getButton(XboxButton.X).whenActive(new ArmToKnownAngle(arm, Arm.ArmPosition.Floor));
        */


        xbox.getDpad().up().whileActive(new MoveUp(arm));
        xbox.getDpad().down().whileActive(new MoveDown(arm));

        arm.setDefaultAction(new StayOnAngle(arm));

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
        arm.print();
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
