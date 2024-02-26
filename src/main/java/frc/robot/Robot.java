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
import frc.robot.subSystems.*;
import frc.robot.actions.ArmController;

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
    private ActionGroup shootMoveTake;

    private final XboxController xbox_systems;
    private final XboxController xbox_driver; //driver
    private Climb climb;
    PowerDistribution a = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);



    public Robot(FrcRobotControl robotControl) {
        super(robotControl);
        //double angle = -1.05 * Math.pow(distance, 2) + 11.2 * distance + 18.4;
        SmartDashboard.putNumber("m of x^3",0);
        SmartDashboard.putNumber("m of x^2",-1.05);
        SmartDashboard.putNumber("m of x",11.2);
        SmartDashboard.putNumber("k0",18.4);

        SmartDashboard.putNumber("Horizontal_distance from target",0);

        this.arm = SystemFactory.createArm();
        swerve = SystemFactory.createSwerveSystem();
        intake = SystemFactory.createIntake();
        shooter = SystemFactory.createShooter();
        xbox_driver = getHidInterface().newXboxController(RobotMap.XBOX_DRIVER);
        xbox_systems = getHidInterface().newXboxController(RobotMap.XBOX_SYSTEMS);
        limelight = new Limelight(swerve,arm);
        climb = SystemFactory.createClimb();

        //driver:
        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox_driver));
        xbox_driver.getButton(XboxButton.X).whenActive(new
                LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, true));
        xbox_driver.getButton(XboxButton.A).whenActive(new
                LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, true, true));
      //  xbox_driver.getButton(XboxButton.X).whenActive(new LimelightAutoAlign(limelight,swerve,arm));


        //xbox_systems.getButton(XboxButton.X).whenActive((new MoveDistance(swerve, 1)));
       ///// xbox_systems.getButton(XboxButton.B).whenActive(new LimelightAutoAlignWithDrive(xbox_driver,limelight,swerve,arm));


        //systems:
        arm.setDefaultAction(new ArmController(arm));
        xbox_systems.getButton(XboxButton.B).whenActive(new TakeIn(intake,arm));

      //  xbox_systems.getButton(XboxButton.B).whenActive(new MoveDistance(swerve, 1));

        xbox_systems.getButton(XboxButton.Y).whileActive(new TakeOut(intake,arm,shooter));
        xbox_systems.getButton(XboxButton.A).whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter));
        //xbox_systems.getButton(XboxButton.X).whenActive(new ShootSpeaker(shooter, intake, arm));
        xbox_systems.getButton(XboxButton.X).whenActive(new Shoot(shooter, intake, arm, limelight));

        /*xbox_systems.getButton(XboxButton.RB).whenActive((Actions.instant(() -> shooter.shootAmp())).andThen(Actions.instant(() -> arm.setYesAmp()))
                .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.AMP_ANGLE_FROM_SHOOTER))));*/
        /*xbox_systems.getButton(XboxButton.LB).whenActive((Actions.instant(() -> arm.setNotAmp()))
                .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE))));*/
        xbox_systems.getButton(XboxButton.RB).whenActive(new ShootAMP(shooter, arm));
        xbox_systems.getButton(XboxButton.LB).whenActive(new ShootToSpeaker(shooter, arm, intake).alongWith(new Shoot(shooter, intake, arm, limelight)));


        xbox_systems.getAxis(XboxAxis.RT).asButton(0.8 ,true).whenActive(new SetDefault(arm,shooter,intake, limelight));
        xbox_systems.getAxis(XboxAxis.LT).asButton(0.8 ,true).whenActive(new ClimbUp(climb, arm));

        //ActionGroup shootSpeaker = new TakeIn(intake,arm).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new ShooterSpeaker(shooter, intake, arm)));

        xbox_systems.getDpad().right().whenActive(new Pull_In(intake));
        xbox_systems.getDpad().left().whenActive(new ClimbDown(climb));
        xbox_systems.getDpad().down().whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter).alongWith(new Shoot(shooter, intake, arm, limelight)));
        xbox_systems.getDpad().up().whenActive(Actions.instant(() -> arm.setSetPointAngle(Arm.CLIMB_ANGLE)));

        limelight.setPipline(0);

        SmartDashboard.putNumber("set point distance", 0);

         /*this.moveAndShoot = new MoveDistance(swerve, -1.2).andThen(new LimelightAutoAlign(limelight, swerve))
                .andThen(new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new ShooterSpeaker(shooter, intake,arm));
*/
         /*this.shootAndMove = new LimelightAutoAlign(limelight, swerve).andThen((Actions.instant(() -> arm.setNotAmp()))
                         .andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE)))).andThen(new ShooterSpeaker(shooter, intake,arm)).andThen(Actions.instant(() -> swerve.resetWheels()))
                         .andThen(new TakeIn(intake, arm))
                         .alongWith(new MoveDistance(swerve, -1.2));*/

        this.shootAndMove = new LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, false).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                .alongWith(new Shoot(shooter, intake,arm, limelight))).andThen((Actions.instant(() -> swerve.resetWheels()))
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

        /*this.shootMoveTakeAndShoot = ((Actions.instant(() -> arm.setNotAmp())).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE)))
                        .andThen(Actions.instant(() -> shooter.shootSpeaker())
                        .alongWith(new Shoot(shooter, intake,arm)))).andThen((Actions.instant(() -> swerve.resetWheels()))
                        .andThen(new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)))
                        .andThen(new LimelightAutoAlign(limelight, swerve).andThen((new SetPointAngleByVision(limelight, intake, arm, shooter))
                        .alongWith(new Shoot(shooter, intake,arm))));*/

        this.shootMoveTakeAndShoot = (Actions.instant(() -> swerve.resetWheels()))
                        .andThen(new ShootToSpeaker(shooter, arm, intake).alongWith(new Shoot(shooter, intake,arm, limelight)))
                        .andThen((new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)))
                        .andThen(new LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, false))
                        .andThen((new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new Shoot(shooter, intake,arm, limelight)));


        this.shootMoveTake = Actions.instant(() -> swerve.resetWheels()).andThen(Actions.instant(() -> arm.setNotAmp()).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE)))
                .andThen(Actions.instant(() -> shooter.shootSpeaker())
                .alongWith(new Shoot(shooter, intake, arm, limelight))))
                .andThen((new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)));



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
        //moveDistance.start();

    }

    @Override
    public void teleopPeriodic() {
       // swerve.drive(0.5, 0, 0);
    }

    @Override
    public void autonomousInit() {
        arm.resetPID();
        swerve.resetWheels();
        this.shootMoveTakeAndShoot.start();
        //new MoveDistance(swerve, -2).start();
        //this.shootMoveTake.start();
       // new MoveByPoseY(swerve, 10.33).start();
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
        arm.setNotAmp();
      //  new LimelightAutoAlignWithDrive(xbox_driver, limelight, swerve, arm).start();




    }

    @Override
    public void testPeriodic() {
        double angle2T = limelight.getXAngleToTarget_Speaker();
        SmartDashboard.putNumber("angle2T",angle2T); //check if the value is correct-it may be the wrong one, so switch

        double avgDistance = limelight.getAvgDistance();
        SmartDashboard.putNumber("avg Distance",avgDistance); //it may not work 100% accuratly, i need to tune it when i'm in the room
        SmartDashboard.putNumber("hopefully real distance",limelight.getDisHorizontalToTarget());
        SmartDashboard.putBoolean("see target",limelight.isThereTarget());

        //calculated angle
        double distanceFromTarget = SmartDashboard.getNumber("Horizontal_distance from target",0);
        calculateAngle(distanceFromTarget);

        /*double setPoint = SmartDashboard.getNumber("set point A", Arm.DEF_ANGLE);
        arm.setSetPointAngle(setPoint);*/
        SmartDashboard.putNumber("set point A", arm.getSetPointAngle());
    }
    public double calculateAngle(double distance){


        if(distance == Double.MIN_VALUE) {
            arm.setPositioningNotControlled();
            return Double.MIN_VALUE;
        }
        else {
            double M_x3 = SmartDashboard.getNumber("m of x^3",0);
            double M_x2 =SmartDashboard.getNumber("m of x^2",-1.05);
            double M_x1 =SmartDashboard.getNumber("m of x",11.2);
            double k0 =SmartDashboard.getNumber("k0",18.4);




            double angle =  M_x3 * Math.pow(distance,3) + M_x2 * Math.pow(distance, 2) + M_x1 * distance + k0;
            SmartDashboard.putNumber("calculated angle", angle);
            return angle;
        }
    }


    @Override
    public void robotPeriodic() {
        arm.print();
        limelight.updateRobotPositionByAprilTag();
        swerve.updatePositioning();
        shooter.print();

        SmartDashboard.putNumber("Limelight angle diff to target", limelight.getXAngleToTarget_Speaker());

        SmartDashboard.putNumber("ActualGyroAngle", swerve.getHeadingDegrees());
        SmartDashboard.putNumber("ActualPoseAngle", swerve.getRobotPose().getRotation().getDegrees());
        SmartDashboard.putNumber("ActualAngleToSpeaker", limelight.getXAngleToTarget_Speaker());
        SmartDashboard.putNumber("ActualAngleToSpeaker2", limelight.getAngleToSpeaker());

        SmartDashboard.putNumber("set point A", arm.getSetPointAngle());

        SmartDashboard.putBoolean("IS IN NOTE", intake.isIN());

        swerve.print();

        SmartDashboard.putNumber("Drive Distance", swerve.getDistancePassedMeters());
        SmartDashboard.putNumber("Place Y robot", swerve.getRobotPose().getY());
        SmartDashboard.putNumber("Place X robot", swerve.getRobotPose().getX());
    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
