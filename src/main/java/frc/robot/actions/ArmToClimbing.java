package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.ShooterSystem;

public class ArmToClimbing extends ActionBase {
    private Arm arm;
    private ShooterSystem shooter;
    private UsbCamera camera;
    private VideoSink videoSink;
    public ArmToClimbing(Arm arm, ShooterSystem shooter, UsbCamera camera, VideoSink videoSink){
        this.arm = arm;
        this.shooter = shooter;
        this.camera = camera;
        this.videoSink = videoSink;
        requires(shooter);
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.stop();
        arm.setSetPointAngle(Arm.CLIMB_ANGLE);
        videoSink.setSource(camera);
        control.finish();
    }

    @Override
    public void execute(ActionControl control) {

    }

    @Override
    public void end(FinishReason reason) {

    }
}
