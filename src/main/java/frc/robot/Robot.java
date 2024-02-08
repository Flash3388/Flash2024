package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
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

   private Arm arm;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        this.arm = SystemFactory.createArm();

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
        arm.resetPID();

    }

    @Override
    public void autonomousPeriodic() {
        double setPoint = SmartDashboard.getNumber("set point A", 20);
        arm.setSetPointAngle(setPoint);
    }


    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

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
