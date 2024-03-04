package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
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
}
