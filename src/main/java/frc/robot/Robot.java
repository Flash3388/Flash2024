package frc.robot;

import com.castle.time.Time;
import com.flash3388.flashlib.frc.robot.FrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.DelegatingFrcRobotControl;
import com.flash3388.flashlib.frc.robot.base.iterative.IterativeFrcRobot;
import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxButton;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.actions.ActionGroup;
import com.flash3388.flashlib.scheduling.actions.Actions;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.actions.*;
import frc.robot.subSystems.*;
import frc.robot.actions.ArmController;

import java.util.Optional;

public class Robot extends DelegatingFrcRobotControl implements IterativeFrcRobot {
    private Swerve swerve;
    private Intake intake;
    private ShooterSystem shooter;
    private Limelight limelight;
    private Arm arm;
    private Climb climb;
    private UsbCamera usbCamera;
    private VideoSink videoSink;


    private ActionGroup moveAndShoot;
    private ActionGroup shootAndMove;
    private ActionGroup shootMoveTakeAndShoot;
    private ActionGroup shootMoveTake;
    private ActionGroup spinShootSpinTakeShoot;
    private ActionGroup spinShootMove;
    private ActionGroup moveBackward;
    private ActionGroup side_spinShootMoveBackward;


    private final XboxController xbox_systems; // systems
    private final XboxController xbox_driver; //driver

    public Robot(FrcRobotControl robotControl) {
        super(robotControl);

        this.arm = SystemFactory.createArm();
        swerve = SystemFactory.createSwerveSystem();
        intake = SystemFactory.createIntake();
        shooter = SystemFactory.createShooter();
        xbox_driver = getHidInterface().newXboxController(RobotMap.XBOX_DRIVER);
        xbox_systems = getHidInterface().newXboxController(RobotMap.XBOX_SYSTEMS);
        limelight = new Limelight(swerve);
        climb = SystemFactory.createClimb();
       // this.usbCamera = CameraServer.startAutomaticCapture();
       // this.videoSink = CameraServer.getServer();
       //  videoSink.setSource(null);

        //driver:
        swerve.setDefaultAction(new DriveWithXbox(swerve, xbox_driver));
        xbox_driver.getButton(XboxButton.X).whenActive(new
                LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, true));
        xbox_driver.getButton(XboxButton.A).whenActive(new
                LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, true, true));
        xbox_driver.getAxis(XboxAxis.RT).asButton(0.8 ,true).whenActive(new SetDefault(arm,shooter,intake, limelight));
        xbox_driver.getDpad().up().whenActive(Actions.instant(() -> Swerve.IS_FIELD_RELATIVE = !Swerve.IS_FIELD_RELATIVE));
        xbox_driver.getDpad().down().whenActive(Actions.instant(() -> Swerve.SIGNUM = -Swerve.SIGNUM));
        xbox_driver.getButton(XboxButton.LB).whileActive(new CollectNote(swerve));

        //systems:
        arm.setDefaultAction(new ArmController(arm));
        xbox_systems.getButton(XboxButton.B).whenActive(new TakeIn(intake,arm));
        xbox_systems.getButton(XboxButton.Y).whileActive(new TakeOut(intake,arm,shooter));
        xbox_systems.getButton(XboxButton.A).whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter));
        xbox_systems.getButton(XboxButton.X).whenActive(new Shoot(shooter, intake, arm, limelight));

        xbox_systems.getButton(XboxButton.RB).whenActive(new ShootAMP(shooter, arm));
        xbox_systems.getButton(XboxButton.LB).whenActive(new ShootToSpeaker(shooter, arm, intake).alongWith(new Shoot(shooter, intake, arm, limelight)));


        xbox_systems.getAxis(XboxAxis.RT).asButton(0.8 ,true).whenActive(new SetDefault(arm,shooter,intake, limelight));
        xbox_systems.getAxis(XboxAxis.LT).asButton(0.8 ,true).whenActive(new ClimbUp(climb, arm));


        xbox_systems.getDpad().right().whenActive(new Pull_In(intake));
        xbox_systems.getDpad().left().whenActive(new ClimbDown(climb));
        xbox_systems.getDpad().down().whenActive(new SetPointAngleByVision(limelight,intake,arm, shooter).alongWith(new Shoot(shooter, intake, arm, limelight)));
        xbox_systems.getDpad().up().whenActive(new ArmToClimbing(arm, shooter));

        limelight.setPipline(0);

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
                .alongWith(new MoveDistance(swerve, -1.5, false)));


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
                        .andThen((new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5, false)))
                        .andThen(new LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, false))
                        .andThen((new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new Shoot(shooter, intake,arm, limelight)));


        this.shootMoveTake = Actions.instant(() -> swerve.resetWheels()).andThen(Actions.instant(() -> arm.setNotAmp()).andThen(Actions.instant(() -> arm.setSetPointAngle(Arm.SPEAKER_ANGLE)))
                .andThen(Actions.instant(() -> shooter.shootSpeaker())
                .alongWith(new Shoot(shooter, intake, arm, limelight))))
                .andThen((new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5, false)));

        this.moveBackward = Actions.instant(() -> swerve.resetWheels()).andThen(new MoveDistance(swerve, -1.5, false));

        this.side_spinShootMoveBackward = new LimelightAutoAlignWithDrive(xbox_driver, limelight, swerve, arm, false, false)
                .andThen((new SetPointAngleByVision(limelight, intake, arm,shooter)).alongWith(new Shoot(shooter, intake, arm, limelight)))
                .andThen(Actions.instant(() -> swerve.resetWheels()))
                .andThen(new MoveDistance(swerve, -1.5, true));

       /* this.spinShootSpinTakeShoot = new LimelightAutoAlignWithDrive(xbox_driver, limelight, swerve, arm, false, false)
                .andThen((new SetPointAngleByVision(limelight, intake, arm,shooter)).alongWith(new Shoot(shooter, intake, arm, limelight)))
                .andThen(new StraightToField(limelight, swerve)).andThen(Actions.instant(() -> swerve.resetWheels()))
                .andThen((new TakeIn(intake, arm)).alongWith(new MoveDistance(swerve, -1.5)))
                .andThen(new LimelightAutoAlignWithDrive(xbox_driver, limelight,swerve,arm, false, false))
                .andThen((new SetPointAngleByVision(limelight, intake, arm, shooter)).alongWith(new Shoot(shooter, intake,arm, limelight)));
      */

         this.spinShootSpinTakeShoot = new LimelightAutoAlignWithDrive(xbox_driver, limelight, swerve, arm, false, false)
                .andThen((new SetPointAngleByVision(limelight, intake, arm,shooter)).alongWith(new Shoot(shooter, intake, arm, limelight)));
        SmartDashboard.putNumber("Which auto?", 1);
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
        arm.resetPID();
        shooter.resetI();
        limelight.init();
        swerve.resetCurrentAngle();
        arm.setNotAmp();
        arm.setSetPointAngle(Arm.DEF_ANGLE);

        //PortForwarder.add(5809, "wpilibpi.local", 5809);

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (!allianceOptional.isEmpty()) {
            DriverStation.Alliance alliance = allianceOptional.get();
            if(alliance == DriverStation.Alliance.Blue)
                Swerve.SIGNUM = -1;
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        arm.resetPID();
        swerve.resetWheels();
        limelight.init();

        swerve.resetCurrentAngle();
        arm.setNotAmp();
        arm.setSetPointAngle(Arm.DEF_ANGLE);

        double auto = SmartDashboard.getNumber("Which auto?", 1);
        if(auto == 1)
            this.shootMoveTakeAndShoot.start();
        else if(auto == 2)
            this.spinShootSpinTakeShoot.start();
        else if(auto == 3)
            this.moveBackward.start();
        if(auto == 4)
            this.side_spinShootMoveBackward.start();
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
        swerve.print();

        SmartDashboard.putNumber("ActualGyroAngle", swerve.getHeadingDegrees());
        SmartDashboard.putNumber("Drive Distance", swerve.getDistancePassedMeters());
        SmartDashboard.putNumber("Place Y robot", swerve.getRobotPose().getY());
        SmartDashboard.putNumber("Place X robot", swerve.getRobotPose().getX());
        SmartDashboard.putNumber("ActualPoseAngle", swerve.getRobotPose().getRotation().getDegrees());

        SmartDashboard.putNumber("AngleToSpeaker", limelight.getXAngleToTarget_Speaker());
        SmartDashboard.putBoolean("see target",limelight.isThereTarget());
        double avgDistance = limelight.getAvgDistance();
        SmartDashboard.putNumber("avg Distance",avgDistance); //it may not work 100% accuratly, i need to tune it when i'm in the room
        SmartDashboard.putNumber("hopefully real distance",limelight.getDisHorizontalToTarget());

        SmartDashboard.putNumber("set point A", arm.getSetPointAngle());
        SmartDashboard.putBoolean("IS IN NOTE", intake.isIN());

    }

    @Override
    public void robotStop() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
