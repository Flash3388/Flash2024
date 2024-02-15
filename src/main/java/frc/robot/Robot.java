package frc.robot;

import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.actions.ActionGroup;
import com.flash3388.flashlib.scheduling.actions.Actions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.*;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.ShooterSystem;
import frc.robot.subSystems.Swerve;
import frc.robot.actions.ArmController;
import frc.robot.subSystems.Arm;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private Intake intake;
    private ShooterSystem shooter;
    private Limelight limelight;

    private final XboxController xbox_systems;
    private final XboxController xbox_driver; //driver
    PowerDistribution a = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);


   private Arm arm;

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        this.arm = SystemFactory.createArm();
        swerve = SystemFactory.createSwerveSystem();
        intake = SystemFactory.createIntake();
        shooter = SystemFactory.createShooter();
        xbox_driver = getHidInterface().newXboxController(RobotMap.XBOX_DRIVER);
        xbox_systems = getHidInterface().newXboxController(RobotMap.XBOX_SYSTEMS);
        limelight = new Limelight(swerve);

        //driver:
        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox_driver));
        xbox_driver.getButton(XboxButton.X).whenActive(new LimelightAutoAlign(limelight,swerve));

        //systems:
        arm.setDefaultAction(new ArmController(arm));
        xbox_systems.getButton(XboxButton.B).whenActive(new TakeIn(intake,arm));
        xbox_systems.getButton(XboxButton.Y).whileActive(new TakeOut(intake,arm,shooter));
        xbox_systems.getButton(XboxButton.A).whenActive(new SetPointAngleByVision(limelight,intake,arm, xbox_systems));

        //ask maayan about it
        xbox_systems.getDpad().down().whenActive(Actions.instant(() -> arm.setSetPointAngle(Arm.DEF_ANGLE)).alongWith(Actions.instant(() -> arm.setNotAmp())));

       // xbox_systems.getAxis(XboxAxis.LT)(new SetDefault(arm, shooter, intake));
        xbox_systems.getAxis(XboxAxis.RT).asButton(0.8 ,true).whenActive(new SetDefault(arm,shooter,intake));

        xbox_systems.getButton(XboxButton.X).whenActive(new ShooterSpeaker(shooter, intake, arm));
        xbox_systems.getButton(XboxButton.RB).whenActive((Actions.instant(() -> arm.setYesAmp())).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.AMP_ANGLE_FROM_SHOOTER))));
        xbox_systems.getButton(XboxButton.LB).whenActive((Actions.instant(() -> arm.setYesAmp())).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.AMP_ANGLE_FROM_INTAKE))));
//add get to angle of speaker
        ActionGroup shootSpeaker = new TakeIn(intake,arm).andThen((new SetPointAngleByVision(limelight, intake, arm, xbox_systems)).alongWith(new ShooterSpeaker(shooter, intake, arm)));
        xbox_systems.getDpad().up().whenActive(shootSpeaker);
        limelight.setPipline(2);

        xbox_systems.getAxis(XboxAxis.LT).asButton(0.8 ,true).whenActive(Actions.instant(() -> arm.setSetPointAngle(calculateAngle(limelight.getDisHorizontalToTarget()))));
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
        arm.resetPID();
        shooter.resetI();
        limelight.init();
    }

    @Override
    public void testPeriodic() {
        double angle2T = limelight.getXAngleToTarget();
        SmartDashboard.putNumber("angle2T",angle2T); //check if the value is correct-it may be the wrong one, so switch

        double distance = limelight.getDistanceToTarget();
        double avgDistance = limelight.getAvgDistance();
        SmartDashboard.putNumber("distance",distance); //it may not work 100% accuratly, i need to tune it when i'm in the room
        SmartDashboard.putNumber("avg Distance",avgDistance); //it may not work 100% accuratly, i need to tune it when i'm in the room


        double actud= Math.cos((limelight.getYAngleToTarget()+65)/distance);
        SmartDashboard.putNumber("actual distance",actud); //it may not work 100% accuratly, i need to tune it when i'm in the room

        double cameraHeight = 0.485;
        double actualDis = Math.sqrt(Math.pow(avgDistance,2) - Math.pow(limelight.getTargetHeight() - cameraHeight,2));
        SmartDashboard.putNumber("hopefully real distance",actualDis);

        //double setPoint = SmartDashboard.getNumber("set point A", Arm.DEF_ANGLE);
        SmartDashboard.putNumber("set point A", arm.getSetPointAngle());
       // arm.setSetPointAngle(setPoint);

        SmartDashboard.putBoolean("see target",limelight.isThereTarget());

       /* SmartDashboard.putNumber("rotation",swerve.getPose2D().getRotation().getRadians());
        SmartDashboard.putNumber("xTrans",swerve.getPose2D().getTranslation().getX());
        SmartDashboard.putNumber("yTrans",swerve.getPose2D().getTranslation().getY());*/

        shooter.changePidValues();
      //  shooter.setVelocity(SmartDashboard.getNumber("set point velocity", 0));
    }
    public double calculateAngle(double distance){

        if(distance == Double.MIN_VALUE) {
            arm.setPositioningNotControlled();
            return Double.MIN_VALUE;
        }
        else {
            double angle = -1.05 * Math.pow(distance, 2) + 11.2 * distance + 18.4;
            return angle;
        }

    }


    @Override
    public void robotPeriodic() {
        arm.print();
        limelight.updateRobotPositionByAprilTag();
        swerve.updateOdometer();
        shooter.print();


       // module – The CAN ID of the PDP/PDH. moduleType – Module type (CTRE or REV
/*
        double currentMaster = a.getCurrent(18);
        double currentFollower = a.getCurrent(19);

        if (Math.abs(currentMaster - currentFollower) > 3) {
            DriverStation.reportWarning("Current difference between arm master and follower", false);
        }

 */

        SmartDashboard.putBoolean("IS IN NOTE", intake.isIN());
        SmartDashboard.putBoolean("IS AGAM", intake.isInAgam());

        swerve.print();
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
