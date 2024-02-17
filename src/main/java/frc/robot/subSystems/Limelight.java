package frc.robot.subSystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.Subsystem;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import com.jmath.ExtendedMath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
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
        double aprilTagId = 7; //default is blue alliance
        if(layout.getOrigin().equals(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)) //if are we red alliance
            aprilTagId =4;
        Pose2d differenceBetweenRobotToTarget;
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
            return robotPoseTargetSpace[5];


         //   return table.getEntry("tx").getDouble(0.0);
        }
        //if i can't see target-use odometer


        differenceBetweenRobotToTarget = apriltagPose.get().toPose2d().relativeTo(swerve.getOdometer().getPoseMeters());
        return differenceBetweenRobotToTarget.getRotation().getDegrees();
    }
    public double getXAngleToTarget_Amp() {// for amp degrees
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        double aprilTagId = 6; //default is blue alliance
        if(layout.getOrigin().equals(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)) //if are we red alliance
            aprilTagId =5;

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
            return robotPoseTargetSpace[5];


            //   return table.getEntry("tx").getDouble(0.0);
        }
        //if i can't see target-use odometer


        Pose2d differenceBetweenRobotToTarget = apriltagPose.get().toPose2d().relativeTo(swerve.getOdometer().getPoseMeters());
        return differenceBetweenRobotToTarget.getRotation().getDegrees();
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

        Pose2d differenceBetweenRobotToTarget = apriltagPose.get().toPose2d().relativeTo(swerve.getOdometer().getPoseMeters());
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
        double cameraHeight = 0.485;
        double actualDis = 0;
        if(getAvgDistance()!=0) {
          //  actualDis = Math.sqrt(Math.pow(getAvgDistance(), 2) - Math.pow(getTargetHeight() - cameraHeight, 2));
            actualDis = getAvgDistance();
        }
        else {
            //relativeTo(robot)
            double aprilTagId = 10; // id of speaker    LimelightHelpers.getFiducialID("limelight-banana");
            SmartDashboard.putNumber("aprilTagId",aprilTagId);
            Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag

            Pose2d differenceBetweenRobotToTarget = swerve.getOdometer().getPoseMeters().relativeTo(apriltagPose.get().toPose2d());
            actualDis = Math.sqrt(Math.pow(differenceBetweenRobotToTarget.getX(),2) + Math.pow(differenceBetweenRobotToTarget.getY(),2));
        }
         SmartDashboard.putNumber("hopefully real distance",actualDis);
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
        if (!isThereTarget()) {
            SmartDashboard.putBoolean("aprilTagPresent",false);
            return;
        }

        double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPose.isPresent()) {
            SmartDashboard.putBoolean("aprilTagPresent", true);
            Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana");
            swerve.setOdometer(robotPose);
        }
    }
















}
