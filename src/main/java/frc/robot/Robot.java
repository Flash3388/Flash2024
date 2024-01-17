package frc.robot;

import com.flash3388.flashlib.control.Direction;
import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.actions.Action;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.DriveWithXbox;
import frc.robot.actions.TrajectoryFollowingAction;
import frc.robot.subSystems.Swerve;
import frc.robot.sysid.SysIdRoutine;
import frc.robot.sysid.SysIdRoutineConfig;
import frc.robot.sysid.SysIdRoutineMechanism;
import frc.robot.subSystems.TrajectoryExample;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {

    private Swerve swerve;
    private XboxController xbox;

    private final SysIdRoutine swerveSysId;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        swerve = SystemFactory.createSwerveSystem();
        swerveSysId = new SysIdRoutine(
                "swerve",
                new SysIdRoutineConfig(),
                new SysIdRoutineMechanism(
                        swerve::sysidDrive,
                        swerve::sysidLog,
                        swerve,
                        "swerve"
                ));

        this.xbox = getHidInterface().newXboxController(RobotMap.XBOX);

        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox));

        //swerveSysId.quasistatic(Direction.FORWARD).start();
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
