package frc.robot.subSystems;

import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightHelpers;

import java.util.Optional;

public class Limelight extends Subsystem {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    private double[] cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
    //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)

    private AprilTagFieldLayout layout;
    private Swerve swerve;

    public Limelight(Swerve swerve){
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide); //if we're on the blue side

        this.swerve = swerve;
    }
    public void setPipline(int n){
        table.getEntry("pipeline").setValue(n);
    }
    public double getXAngleToTarget() {////
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        if (isThereTarget()) {
            cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
            SmartDashboard.putNumber("cameraPtoTRotation 5", cameraPoseTargetSpace[5]);
            SmartDashboard.putNumber("cameraPtoTRotation 4", cameraPoseTargetSpace[4]);
            SmartDashboard.putNumber("cameraPtoTRotation 2", cameraPoseTargetSpace[2]);
            SmartDashboard.putNumber("cameraPtoTRotation 1", cameraPoseTargetSpace[1]);
            SmartDashboard.putNumber("cameraPtoTRotation 0", cameraPoseTargetSpace[0]);
            SmartDashboard.putNumber("cameraPtoTRotation 3", cameraPoseTargetSpace[3]);
            SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0));

            return table.getEntry("tx").getDouble(0.0);

            // return table.getEntry("tx").getDouble(0.0);
        }
        return 0;
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
            cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
            SmartDashboard.putNumber("cameraPtoTRotation 4", cameraPoseTargetSpace[4]);
            SmartDashboard.putNumber("ty", table.getEntry("ty").getDouble(0.0));

            return cameraPoseTargetSpace[4];
        }
        return 0;
    }



    public double getDistanceToTarget() {
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        if (isThereTarget()) {
            cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
            double distance = Math.sqrt(
                    Math.pow(cameraPoseTargetSpace[0], 2) +
                            Math.pow(cameraPoseTargetSpace[1], 2) +
                            Math.pow(cameraPoseTargetSpace[2], 2)
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
        double actualDis = Math.sqrt(Math.pow(getAvgDistance(),2) - Math.pow(getTargetHeight() - cameraHeight,2));
        SmartDashboard.putNumber("hopefully real distance",actualDis);
        return actualDis;
    }
    public double getAvgDistance(){
        double reading=getDistanceToTarget();
        if(reading!=0){
            readings[numOfReadings%10]=reading;
            numOfReadings++;
        }
        else numOfNoTargetDetection++;

        if(numOfNoTargetDetection == 50){ //if we are not seeing the target for a while -> the measurements are wrong by now
            readings = new double[Window_size]; //initialize the array
            numOfNoTargetDetection = 0;
        }

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
