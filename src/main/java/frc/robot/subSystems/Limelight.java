package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.jmath.vectors.Vector2;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class Limelight extends Subsystem {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    private double[] robotPoseTargetSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight-banana");

    private AprilTagFieldLayout layout;
    private Swerve swerve;
    private Arm arm;
    private static final double DELAY_BEFORE_FINISH_IN_SECONDS = 2;
    private Timer timer;
    private int Window_size =10;
    private double[] readings = new double[Window_size];
    public int numOfReadings = 0;
    public double sum = 0;

    private double accuracyInVision = 100; //percents
    public static boolean KEEP_UPDATING_ODOMETER = true;


    public Limelight(Swerve swerve){
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide); //always
        timer = new Timer();
        this.swerve = swerve;


    }
    public void setPipline(int n){
        table.getEntry("pipeline").setValue(n);
    }

    public double getXAngleToTarget_Speaker() {// for speaker
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 7; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId =4;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose3d apriltagPose = apriltagPoseOptional.get();
        Pose2d robotPose = swerve.getRobotPose();

        double deltaX = apriltagPose.getX() - robotPose.getX();
        double deltaY = apriltagPose.getY() - robotPose.getY();
        double angleToSpeakerRad= Math.atan2(deltaY,deltaX);
        double angleToSpeakerDeg= Math.toDegrees(angleToSpeakerRad);
        double angleFromRobotToSpeaker = angleToSpeakerDeg - robotPose.getRotation().getDegrees();
        //normalize the angles
        if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        return angleFromRobotToSpeaker;
    }

    public double getAngleToSpeaker() {
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 7; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId =4;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose2d aprilTag = apriltagPoseOptional.get().toPose2d();
        Pose2d robotPose = swerve.getRobotPose();
        Twist2d twist = aprilTag.rotateBy(Rotation2d.fromDegrees(180)).log(robotPose);
        double angleDiffBase = Math.toDegrees(twist.dtheta);
        double coordDiff = new Vector2(robotPose.getX(), robotPose.getY()).angleTo(new Vector2(aprilTag.getX(), aprilTag.getY()));

        return angleDiffBase + coordDiff;
    }

    public double getXAngleToTarget_Amp() {// for amp degrees

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 6; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId = 5;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose3d apriltagPose = apriltagPoseOptional.get();
        Pose2d robotPose = swerve.getRobotPose();

        double angleFromRobotToSpeaker = 90 - robotPose.getRotation().getDegrees();
        //normalize the angles
       if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        return angleFromRobotToSpeaker;
    }
    public double getXDistanceToTarget_Amp(){
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 6; //default is red alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if we are blue alliance-limelight towards us
            aprilTagId = 5;
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId));
        double distance = Math.sqrt(Math.pow(swerve.getRobotPose().getX() - apriltagPose.get().getX(), 2));
        if(swerve.getRobotPose().getX() - apriltagPose.get().getX() > 0) //if robot is to the right of the amp
            distance *= -1;
        return distance;
    }

    public double getTargetHeight() {
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        if (isThereTarget()) {
            double height = 0;
            double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
            SmartDashboard.putNumber("aprilTagId",aprilTagId);
            Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag
            if (apriltagPose.isPresent()){
                height = apriltagPose.get().getZ();
            }

            return height;
        }
        return 0;
    }

    public double getDistanceToTarget() {
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        if (isThereTarget()) {
            robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
            double distance = Math.sqrt(
                    Math.pow(robotPoseTargetSpace[0], 2) +
                            Math.pow(robotPoseTargetSpace[1], 2) +
                            Math.pow(robotPoseTargetSpace[2], 2)
            );
            return distance;
        }
        return 0;
    }

    public void init() {
        numOfReadings = 0;
    }

    public double getDisHorizontalToTarget(){
        double cameraHeight = 0.485;
        double actualDis = 0;
       /* if(getAvgDistance()!=0) {
            if(getAvgDistance() > 4)
                actualDis = getAvgDistance();
            else
              actualDis = Math.sqrt(Math.pow(getAvgDistance(), 2) - Math.pow(getTargetHeight() - cameraHeight, 2));
        }
        else {*/
            //relativeTo(robot)

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 7; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId = 4;

        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag

        actualDis = Math.sqrt(Math.pow(swerve.getRobotPose().getX()-apriltagPose.get().getX(),2) + Math.pow(swerve.getRobotPose().getY()- apriltagPose.get().getY(),2));

        return actualDis - 0.225; //0.225
    }
    public double getAvgDistance(){
        double reading=getDistanceToTarget();
        if(reading!=0){ //see
            readings[numOfReadings%10]=reading;
            numOfReadings++;
            timer.reset();
            timer.start();
        }
        else{ //can't see
            if (timer.hasElapsed(5)) {
                readings = new double[Window_size];}
        }

        SmartDashboard.putNumber("no target Time counted", timer.get());

        int loopSize = Window_size;
        if(numOfReadings<Window_size){
            loopSize = numOfReadings;
        }
        sum = 0;
        for (int i = 0; i < loopSize ; i++) {
            sum += readings[i];
        }

        double avg=0;
        if(loopSize>0){
            avg=sum/loopSize;
        }
        return avg;
    }
    public void stopTimer(){
        timer.stop();
    }
    public void startTimer(){
        timer.start();
    }

    public boolean isThereTarget(){
        return LimelightHelpers.getTV("limelight-banana"); //tv=1.0 means a target is detected
    }
    public double angleToForward_FieldRelative_Odometer(){
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double angle = 0; //default is red alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Blue) //if we are blue alliance-limelight towards us
            angle +=180;

        SmartDashboard.putNumber("angle to field forward", angle);
        return angle;

    }

    public void updateRobotPositionByAprilTag(){
        if (!isThereTarget() || getAvgDistance() > 3 || !Limelight.KEEP_UPDATING_ODOMETER) {  /*|| getAvgDistance() >= 2.5*/
            SmartDashboard.putBoolean("aprilTagPresent",false);
            return;
        }
        if(getAvgDistance() <= 1.6)
            setPipline(1);
        else //2-3.5 m
            setPipline(0);

        double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId));
        if (apriltagPose.isPresent()) {
            SmartDashboard.putBoolean("aprilTagPresent", true);
            Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana");

        /*    double latency = LimelightHelpers.getLatency_Capture("limelight-banana") +
                    LimelightHelpers.getLatency_Pipeline("limelight-banana"); */
            double timestamp = Timer.getFPGATimestamp();// - latency;
            swerve.updatePositionFromVision(robotPose, timestamp); //check how to correctly check the limelight time
        }
    }
         /* public double getXAngleToTarget_Speaker_TomWay() {// for speaker
            //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
            Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
            if (allianceOptional.isEmpty()) {
                return 0;
            }

            double aprilTagId = 4; //default is blue alliance - 7 is the correct one
            DriverStation.Alliance alliance = allianceOptional.get();
            if(alliance == DriverStation.Alliance.Red) //if are we red alliance
                aprilTagId =4;

            Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
            if (apriltagPoseOptional.isEmpty()) {
                return 0;
            }

            Pose3d apriltagPose = apriltagPoseOptional.get();
            Pose2d robotPose = swerve.getRobotPose();

            Pose2d rotateAprilTag = apriltagPose.toPose2d().rotateBy(Rotation2d.fromDegrees(180));
            Twist2d twist = robotPose.log(rotateAprilTag);
            double rotationDiff = Math.toDegrees(twist.dtheta);

            Vector2 robotV = new Vector2(robotPose.getX(),robotPose.getY());
            Vector2 aprilTagV = new Vector2(apriltagPose.getX(),apriltagPose.getY());
            double posVecDiff = Math.toDegrees(Math.acos((aprilTagV.dot(robotV) / robotV.magnitude() * aprilTagV.magnitude())));

            return posVecDiff * rotationDiff;
        } */

}
