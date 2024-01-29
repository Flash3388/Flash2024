package frc.robot.subsystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import frc.robot.LimelightHelpers;

public class Limelight extends Subsystem {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    /*
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

     */

    public Limelight(){

    }
    public void setPipline(int n){
        table.getEntry("pipeline").setValue(n);
    }
    public double getXAngleToTarget() {
        return table.getEntry("tx").getDouble(0.0);
    }
    public double getDistanceToTarget(){
        double[] cameraPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("limelight-banana");
        //(Xpos, Ypos, Zpos, Xrot, Yrot, Zrot)
        double distance = Math.sqrt(
                Math.pow(cameraPoseTargetSpace[0],2) +
                Math.pow(cameraPoseTargetSpace[1],2) +
                Math.pow(cameraPoseTargetSpace[2],2)
        );
        return distance;
    }
















}
