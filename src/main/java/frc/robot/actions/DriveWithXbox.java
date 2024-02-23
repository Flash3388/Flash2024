package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Swerve;

public class DriveWithXbox extends ActionBase {
    private Swerve swerve;
    private XboxController xbox_driver;

    public DriveWithXbox(Swerve swerve, XboxController xbox_driver){
        this.swerve = swerve;
        this.xbox_driver = xbox_driver;
        requires(swerve);
    }

    @Override
    public void initialize(ActionControl control) {

    }

    @Override
    public void execute(ActionControl control) {
        double driveY = -xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble() ;
        double driveX = -xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
        double rotation = -xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble();

        driveY = Math.abs(driveY) > 0.2 ? driveY * Swerve.MAX_SPEED    : 0;
        driveX = Math.abs(driveX) > 0.2 ? driveX  * Swerve.MAX_SPEED : 0;
        rotation = Math.abs(rotation) > 0.2 ? rotation * Swerve.MAX_SPEED : 0;

        boolean isFiledRelative = SmartDashboard.getBoolean("Is Field Relative?", true);
        this.swerve.drive(driveY /2 ,driveX / 2 ,rotation/2, true );
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
