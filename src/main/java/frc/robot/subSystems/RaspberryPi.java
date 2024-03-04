package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RaspberryPi extends Subsystem {

    private PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public RaspberryPi(){
        camera.setPipelineIndex(0);
    }

    public double getXAngleToTarget(){
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        if(!pipelineResult.hasTargets())
            return 0;
        PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
        return bestTarget.getYaw();
    }

    public boolean hasTargets(){
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        return pipelineResult.hasTargets();
    }

    public void print(){
        SmartDashboard.putBoolean("Raspberry Pi Has Targets", hasTargets());
        SmartDashboard.putNumber("Raspberry Pi Has Targets", getXAngleToTarget());
    }
}
