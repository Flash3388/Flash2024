package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxAxis;
import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Swerve;

import java.util.Optional;

public class DriveWithXbox extends ActionBase {
    private Swerve swerve;
    private XboxController xbox_driver;
    private double signum = 1;

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

        double driveY = Swerve.SIGNUM * xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble();
        double driveX = Swerve.SIGNUM * xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
        double rotation = -xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble();

        driveX = Math.abs(driveX) > 0.05 ? driveX * driveX * Math.signum(driveX) * Swerve.MAX_SPEED : 0;
        driveY = Math.abs(driveY) > 0.05 ? driveY * driveY * Math.signum(driveY) * Swerve.MAX_SPEED : 0;
        rotation = Math.abs(rotation) > 0.05 ? rotation * Math.signum(rotation) * rotation * Swerve.MAX_SPEED : 0;


      //  boolean isFiledRelative = SmartDashboard.getBoolean("Is Field Relative?", false);// "Is Field Relative?", false
        SmartDashboard.putBoolean("Is Field Relative?", Swerve.IS_FIELD_RELATIVE);// "Is Field Relative?", false
        this.swerve.drive(driveY  ,driveX  ,rotation, Swerve.IS_FIELD_RELATIVE);
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
