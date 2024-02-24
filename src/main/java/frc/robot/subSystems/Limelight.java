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
    //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)

    private AprilTagFieldLayout layout;
    private Swerve swerve;
    private Arm arm;
    private static final double DELAY_BEFORE_FINISH_IN_SECONDS = 2;
    private Timer timer;
    private final LinearFilter filterX = LinearFilter.movingAverage(20);
    private final LinearFilter filterY = LinearFilter.movingAverage(20);
    private final LinearFilter filterAngle = LinearFilter.movingAverage(20);

    private double accuracyInVision = 100; //percents


    public Limelight(Swerve swerve, Arm arm){
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide); //if we're on the blue side
        timer = new Timer();
        this.swerve = swerve;
        this.arm = arm;

    }
    public void setPipline(int n){
        table.getEntry("pipeline").setValue(n);
    }

    public double getXAngleToTarget_Speaker() {// for speaker
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

        double deltaX = apriltagPose.getX() - robotPose.getX();
        double deltaY = apriltagPose.getY() - robotPose.getY();
        double angleToSpeakerRad= Math.atan2(deltaY,deltaX);
        double angleToSpeakerDeg= Math.toDegrees(angleToSpeakerRad);
        double angleFromRobotToSpeaker = angleToSpeakerDeg - robotPose.getRotation().getDegrees();
        //normalize the angles
        if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        SmartDashboard.putNumber("angle to speaker", angleFromRobotToSpeaker);



        getDisHorizontalToTarget();
        return angleFromRobotToSpeaker;
    }

    public double getAngleToSpeaker() {
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

        Pose2d aprilTag = apriltagPoseOptional.get().toPose2d();
        Pose2d robotPose = swerve.getRobotPose();
        Twist2d twist = aprilTag.rotateBy(Rotation2d.fromDegrees(180)).log(robotPose);
        double angleDiffBase = Math.toDegrees(twist.dtheta);
        double coordDiff = new Vector2(robotPose.getX(), robotPose.getY()).angleTo(new Vector2(aprilTag.getX(), aprilTag.getY()));

        return angleDiffBase + coordDiff;
    }

    public double getXAngleToTarget_Amp() {// for amp degrees
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)


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
    public double getXAngleToTarget_Climbing() {
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        Pose2d robotPose = swerve.getRobotPose();
        DriverStation.Alliance alliance = allianceOptional.get();
        double aprilTagId; //random

        double yLength_BetweenDriversWall_HorizontalStage = 0.02539999 * (121.0 + 106.19); // inches * 0.02539999 = meters
        double XLength_BetweenSideWalls_SideStage = layout.getFieldLength() / 2; //in meters

        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
        {
            if(robotPose.getY() < layout.getFieldWidth() - yLength_BetweenDriversWall_HorizontalStage)
                aprilTagId = 13;
            else if(robotPose.getX() > XLength_BetweenSideWalls_SideStage)
                aprilTagId = 12;
            else if (robotPose.getX() < XLength_BetweenSideWalls_SideStage) {
                aprilTagId = 11;
            }
            else return 0;
        }
        else { //blue alliance
            if(robotPose.getY() > yLength_BetweenDriversWall_HorizontalStage)
                aprilTagId = 14;
            else if(robotPose.getX() > XLength_BetweenSideWalls_SideStage)
                aprilTagId = 15;
            else if (robotPose.getX() < XLength_BetweenSideWalls_SideStage) {
                aprilTagId = 16;
            }
            else return 0;
        }

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose3d apriltagPose = apriltagPoseOptional.get();

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



    public double getXDistanceToTarget_Amp() {// for amp distance
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        double aprilTagId = 6; //default is blue alliance
        if(layout.getOrigin().equals(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)) //if are we red alliance
            aprilTagId =5;

        //the movement is in the x axis
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (isThereTarget()) {

            robotPoseTargetSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight-banana");
            SmartDashboard.putNumber("cameraPtoTRotation 5", robotPoseTargetSpace[5]);
            SmartDashboard.putNumber("cameraPtoTRotation 4", robotPoseTargetSpace[4]);
            SmartDashboard.putNumber("cameraPtoTRotation 2", robotPoseTargetSpace[2]);
            SmartDashboard.putNumber("cameraPtoTRotation 1", robotPoseTargetSpace[1]);
            SmartDashboard.putNumber("cameraPtoTRotation 0", robotPoseTargetSpace[0]);
            SmartDashboard.putNumber("cameraPtoTRotation 3", robotPoseTargetSpace[3]);
            SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0));

            //check if it works
            return robotPoseTargetSpace[0];
        }
        //if i can't see target-use odometer

        Pose2d differenceBetweenRobotToTarget = apriltagPose.get().toPose2d().relativeTo(swerve.getRobotPose());
        if(differenceBetweenRobotToTarget.getX() > 2) return 0; //if it's too far-probably pressened by mistake
        return differenceBetweenRobotToTarget.getX(); //hoping it'll work-for both the positive and negative side
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
    public double getYAngleToTarget() {////
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        if (isThereTarget()) {
            robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
            SmartDashboard.putNumber("cameraPtoTRotation 4", robotPoseTargetSpace[4]);
            SmartDashboard.putNumber("ty", table.getEntry("ty").getDouble(0.0));

            return robotPoseTargetSpace[4];
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
    private int Window_size =10;
    private double[] readings = new double[Window_size];
    public int numOfReadings = 0;
    public double sum = 0;
    public int numOfNoTargetDetection = 0;

    public void init() {
        numOfReadings = 0;
        numOfNoTargetDetection = 0;
    }

    public double getDisHorizontalToTarget(){
        // change the id according to which alliance i'm in
        double cameraHeight = 0.485;
        double actualDis = 0;
        double avgDis = getAvgDistance();
        if(avgDis!=0) {
            if(avgDis > 4)
                actualDis = avgDis;
            else
              actualDis = Math.sqrt(Math.pow(avgDis, 2) - Math.pow(getTargetHeight() - cameraHeight, 2));
        }


        double aprilTagId = 4; // id of speaker    LimelightHelpers.getFiducialID("limelight-banana");
        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag


        double odometerDis = Math.sqrt(Math.pow(swerve.getRobotPose().getX() - apriltagPose.get().getX() , 2) + Math.pow(swerve.getRobotPose().getY() - apriltagPose.get().getY(),2));
//add auto align to climbing

        /*
        else {
            //relativeTo(robot)
            double aprilTagId = 4; // id of speaker    LimelightHelpers.getFiducialID("limelight-banana");
            SmartDashboard.putNumber("aprilTagId",aprilTagId);
            Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag

            //not sure if it'll work

            Pose2d differenceBetweenRobotToTarget = swerve.getRobotPose().relativeTo(apriltagPose.get().toPose2d());
            actualDis = Math.sqrt(Math.pow(differenceBetweenRobotToTarget.getX(),2) + Math.pow(differenceBetweenRobotToTarget.getY(),2));
        }

         */
         SmartDashboard.putNumber("odometry distance",odometerDis);
         SmartDashboard.putNumber("actualDis",actualDis);
        return actualDis;

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

        SmartDashboard.putNumber(" no target Time counted", timer.get());



      /*  if(numOfNoTargetDetection == 50){ //if we are not seeing the target for a while -> the measurements are wrong by now
            readings = new double[Window_size]; //initialize the array
            numOfNoTargetDetection = 0;
        }
*/
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


      /*
      Yaron's way. didn't quite work.
        double reading=getDistanceToTarget();
        if(reading!=0){
            sum += reading;
            if (numOfReadings > Window_size) {
                sum -= readings[numOfReadings % 10];
            }
            readings[numOfReadings%10]=reading;

            numOfReadings++;
        }
        int loopSize = Window_size;
        if(numOfReadings<=Window_size){
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

       */
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
    public void updateRobotPositionByAprilTag(){
        if (!isThereTarget() || getAvgDistance() >= 2) { //2.8
            SmartDashboard.putBoolean("aprilTagPresent",false);
            return;
        }

        double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPose.isPresent()) {
            SmartDashboard.putBoolean("aprilTagPresent", true);
            Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana");

            //double newX = filterX.calculate(robotPose.getX());
            //double newY = filterY.calculate(robotPose.getY());
            //double newAngle = filterAngle.calculate(robotPose.getRotation().getDegrees());
            //robotPose = new Pose2d(new Translation2d(newX, newY), Rotation2d.fromDegrees(newAngle));

            double latency = LimelightHelpers.getLatency_Capture("limelight-banana") +
                    LimelightHelpers.getLatency_Pipeline("limelight-banana");
            double timestamp = Timer.getFPGATimestamp();// - latency;
            swerve.updatePositionFromVision(robotPose, timestamp); //check how to correctly check the limelight time
        }
    }

    public double getXAngleToTarget_Speaker_TelNofWay() {// for speaker
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

        double deltaX = apriltagPose.getX() - robotPose.getX();
        double deltaY = apriltagPose.getY() - robotPose.getY();
        double angleToSpeakerRad= Math.atan2(deltaY,deltaX);
        double angleToSpeakerDeg= Math.toDegrees(angleToSpeakerRad);
        double angleFromRobotToSpeaker = angleToSpeakerDeg - robotPose.getRotation().getDegrees();
        //normalize the angles
        if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        return angleFromRobotToSpeaker;




        /* doesn't matter if we see an april tag
        if (isThereTarget()) {

            double deltaX = apriltagPose.getX() - robotPose.getX();
            double deltaY = apriltagPose.getY() - robotPose.getY();
            double angleToSpeakerRad= Math.atan2(deltaY,deltaX);
            double angleToSpeakerDeg= Math.toDegrees(angleToSpeakerRad);
            double angleFromRobotToSpeaker = angleToSpeakerDeg - robotPose.getRotation().getDegrees();
            //normalize the angles
            if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
            else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

            return angleFromRobotToSpeaker;
              */
        }




        public double getXAngleToTarget_Speaker_TomWay() {// for speaker
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
        }













}
