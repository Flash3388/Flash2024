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
    private Arm arm;

    private MoveDistance moveDistance;

    private ActionGroup moveAndShoot;
    private ActionGroup shootAndMove;
    private ActionGroup shootMoveTakeAndShoot;

    private final XboxController xbox_systems;
    private final XboxController xbox_driver; //driver
    PowerDistribution a = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);



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
        xbox_systems.getButton(XboxButton.A).whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter));
        //xbox_systems.getButton(XboxButton.X).whenActive(new ShootSpeaker(shooter, intake, arm));

        /*xbox_systems.getButton(XboxButton.RB).whenActive((Actions.instant(() -> shooter.shootAmp())).andThen(Actions.instant(() -> arm.setYesAmp()))
                .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.AMP_ANGLE_FROM_SHOOTER))));*/
        xbox_systems.getButton(XboxButton.RB).whenActive(new ShootAMP(shooter, arm));

        xbox_systems.getButton(XboxButton.LB).whenActive((Actions.instant(() -> arm.setNotAmp()))
                .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE))));

        xbox_systems.getAxis(XboxAxis.RT).asButton(0.8 ,true).whenActive(new SetDefault(arm,shooter,intake));
        xbox_systems.getAxis(XboxAxis.LT).asButton(0.8 ,true).whenActive
                (Actions.instant(() -> arm.setSetPointAngle(calculateAngle(limelight.getDisHorizontalToTarget()))));


        //ActionGroup shootSpeaker = new TakeIn(intake,arm).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new ShooterSpeaker(shooter, intake, arm)));

        xbox_systems.getDpad().right().whenActive(new Pull_In(intake));
        xbox_systems.getDpad().up().whenActive(new Shoot(shooter, intake, arm));
       // xbox_systems.getDpad().left().whenActive(new SetDefault(arm,shooter,intake));
        xbox_systems.getDpad().down().whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter).alongWith(new Shoot(shooter, intake, arm)));

        limelight.setPipline(2);

        moveDistance = new MoveDistance(swerve);
        SmartDashboard.putNumber("set point distance", 0);

         /*this.moveAndShoot = new MoveDistance(swerve, -1.2).andThen(new LimelightAutoAlign(limelight, swerve))
                .andThen(new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new ShooterSpeaker(shooter, intake,arm));
*/
         /*this.shootAndMove = new LimelightAutoAlign(limelight, swerve).andThen((Actions.instant(() -> arm.setNotAmp()))
                         .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE)))).andThen(new ShooterSpeaker(shooter, intake,arm)).andThen(Actions.instant(() -> swerve.resetWheels()))
                         .andThen(new TakeIn(intake, arm))
                         .alongWith(new MoveDistance(swerve, -1.2));*/

        this.shootAndMove = new LimelightAutoAlign(limelight, swerve).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                .alongWith(new Shoot(shooter, intake,arm))).andThen((Actions.instant(() -> swerve.resetWheels()))
                .andThen(new TakeIn(intake, arm))
                .alongWith(new MoveDistance(swerve, -1.5)));
        // we need to make sure FL wheel is with its gear in

       /* original
       this.shootMoveTakeAndShoot = new LimelightAutoAlign(limelight, swerve).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                .alongWith(new Shoot(shooter, intake,arm))).andThen((Actions.instant(() -> swerve.resetWheels()))
                .andThen(new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)))
                .andThen(new LimelightAutoAlign(limelight, swerve).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                .alongWith(new Shoot(shooter, intake,arm))));
       */

        this.shootMoveTakeAndShoot = (((Actions.instant(() -> arm.setNotAmp())).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE))))
                        .alongWith(new Shoot(shooter, intake,arm))).andThen((Actions.instant(() -> swerve.resetWheels()))
                        .andThen(new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)))
                        .andThen(new LimelightAutoAlign(limelight, swerve).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                        .alongWith(new Shoot(shooter, intake,arm))));


        /*
   write code to detect on which april tag id i'm looking at-so it'll calculate only based on the middle one
   -add time to when i can't see apriltags-10 seconds
    */

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        swerve.resetWheels();
        moveDistance.start();

    }

    @Override
    public void teleopPeriodic() {
       // swerve.drive(0.5, 0, 0);
        SmartDashboard.putNumber("Drive Distance", swerve.getDistancePassedMeters());
    }

    @Override
    public void autonomousInit() {
        arm.resetPID();
        swerve.resetWheels();
        double dist = SmartDashboard.getNumber("set point distance", 0);
        //this.moveAndShoot.start();
        //this.shootAndMove.start();
        this.shootMoveTakeAndShoot.start();
        //new MoveDistance(swerve, -2).start();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void testInit() {
        arm.resetPID();
        shooter.resetI();
        limelight.init();
        swerve.resetWheels();
        swerve.resetCurrentAngle();
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
        SmartDashboard.putNumber("set point A", arm.getSetPointAngle());

        SmartDashboard.putBoolean("IS IN NOTE", intake.isIN());
        SmartDashboard.putBoolean("IS AGAM", intake.isInAgam());

        swerve.print();

        SmartDashboard.putNumber("Drive Distance", swerve.getDistancePassedMeters());
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
