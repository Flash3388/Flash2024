package frc.robot.actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.Swerve;

import java.util.Optional;

public class RotationUntilAprilTag extends ActionBase {
    private Swerve swerve;
    private Limelight limelight;

    private static double speed = 0.2;
    public RotationUntilAprilTag(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;

        requires(swerve, limelight);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {

        double signum = 1;
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (!allianceOptional.isEmpty()) {
            DriverStation.Alliance alliance = allianceOptional.get();
            if(alliance == DriverStation.Alliance.Blue)
                signum = -1;
        }

        speed = signum * speed;

        swerve.drive(0, 0, speed);

        if(limelight.isThereTarget())
            control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
