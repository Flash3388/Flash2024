package frc.robot.subsystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class Limelight extends Subsystem {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    private double[] cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
    //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)

    /*
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

     */
    private AprilTagFieldLayout layout;
    private Swerve swerve;

    public Limelight(Swerve swerve){
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        layout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide); //if we're on the blue side

        this.swerve = swerve; //maybe i'll remove it
    }
    public void setPipline(int n){
        table.getEntry("pipeline").setValue(n);
    }
    public double getXAngleToTarget() {
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        SmartDashboard.putNumber("cameraPtoTRotation",cameraPoseTargetSpace[5]);
        SmartDashboard.putNumber("tx",table.getEntry("tx").getDouble(0.0));

        return cameraPoseTargetSpace[5];

       // return table.getEntry("tx").getDouble(0.0);
    }
    public double getDistanceToTarget(){
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        double distance = Math.sqrt(
                Math.pow(cameraPoseTargetSpace[0],2) +
                Math.pow(cameraPoseTargetSpace[1],2) +
                Math.pow(cameraPoseTargetSpace[2],2)
        );
        return distance;
    }

    public boolean isThereTarget(){
        return LimelightHelpers.getTV("limelight-banana"); //tv=1.0 means a target is detected
    }
    public void CheckTargetAndUpdateOdometer(){
        if(isThereTarget()){
            updateRobotPositionByAprilTag();
        }
    }
    /*
    setOdometer(Rotation2d gyro, Pose2d pose2d) {
        odometer.resetPosition(gyro, getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
     */

    public void updateRobotPositionByAprilTag(){
        if (!isThereTarget()) {
            SmartDashboard.putBoolean("aprilTagPresent",false);
            return;
        }

        Pose3d robotToCamera = LimelightHelpers.getCameraPose3d_RobotSpace("limelight-banana");
        double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
        SmartDashboard.putNumber("aprilTagId",aprilTagId);
        Optional<Pose3d> apriltagPose = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPose.isPresent()) {
            /*Pose3d actualAprilTagPose = apriltagPose.get();
            SmartDashboard.putBoolean("aprilTagPresent",true);
            Pose3d cameraToTarget = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-banana");
            Transform3d cameraToTargetTransform = new Transform3d(cameraToTarget.getTranslation(),cameraToTarget.getRotation());
            //  Rotation3d cameraToTargetRotation = cameraToTargetTransform.getRotation();
            Transform3d robotToCameraTransform = new Transform3d(robotToCamera.getTranslation(),robotToCamera.getRotation());
            // Rotation3d cameraToRobotRotation = cameraToRobot.getRotation();

            Pose3d robotPoseAbsolute = actualAprilTagPose.transformBy(cameraToTargetTransform.inverse()).transformBy(robotToCameraTransform.inverse());
           //  Pose3d robotPoseAbsoluteRotation = actualAprilTagPose.rotateBy(cameraToRobotTransform.getRotation()).rotateBy(cameraToRobotTransform.getRotation());

            SmartDashboard.putString("robotToCamera", robotToCamera.toString());
            SmartDashboard.putString("aprilTagPose", actualAprilTagPose.toString());
            SmartDashboard.putString("cameraToTarget", cameraToTarget.toString());
            SmartDashboard.putString("robotPoseAbsolute", robotPoseAbsolute.toString());*/

            Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana");
            swerve.setOdometer(robotPose);
                /*
                odometer.resetPosition(gyro, getModulePositions(),
                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
                 */














/*



          //  double aprilTagId = LimelightHelpers.getFiducialID("limelight-banana");
            SmartDashboard.putNumber("aprilTagId",aprilTagId);

            Optional<Pose3d> aprilTagPose3d = layout.getTagPose((int)(aprilTagId)); //position of apriltag
            Translation3d translation3d = aprilTagPose3d.get().getTranslation(); //(x,y,z)
            Rotation3d rotation = aprilTagPose3d.get().getRotation(); //?

            //an idea
            // get distance to target won't work because of height difference
            double realDisToTheWall = Math.cos(table.getEntry("ty").getDouble(0.0)) * getDistanceToTarget(); //d2 = cos(b) * d

            double x2 = translation3d.getY() - Math.asin(getXAngleToTarget()) *  realDisToTheWall; // sin(a) = (x1-x2) / d
            double y2 = translation3d.getY() - Math.acos(getXAngleToTarget()) *  realDisToTheWall; // cos(a) = (y1-y2) / d
            // d-distance between camera and aprilTag
            // a-angle between camera and aprilTag


            swerve.setOdometer(new Rotation2d(Math.toRadians(swerve.getHeadingDegrees())),
                    new Pose2d(x2,y2,new Rotation2d(Math.toRadians(rotation.getZ() + swerve.getHeadingDegrees()))));

            //traslating aprilTagAngle from us -> getting gyro angle
            //translating apilTagLocation -> getting Pose2D
                */
        }
    }
















}
