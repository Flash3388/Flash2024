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
      /*  double driveY = -Math.pow(xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble(),2);
        double driveX = -Math.pow(xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble(),2);
        double rotation = -Math.pow(xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble(),2);
    */

        double signum = 1;
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (!allianceOptional.isEmpty()) {
            DriverStation.Alliance alliance = allianceOptional.get();
            if(alliance == DriverStation.Alliance.Blue) //if are we red alliance
                signum = -1;
        }
//4



        double driveY = signum * xbox_driver.getAxis(XboxAxis.LeftStickY).getAsDouble();
        double driveX = signum * xbox_driver.getAxis(XboxAxis.LeftStickX).getAsDouble() ;
        double rotation = -signum * xbox_driver.getAxis(XboxAxis.RightStickX).getAsDouble();

        driveX = Math.abs(driveX) > 0.2 ? driveX * Swerve.MAX_SPEED : 0;
        driveY = Math.abs(driveY) > 0.2 ? driveY  * Swerve.MAX_SPEED : 0;
        rotation = Math.abs(rotation) > 0.2 ? rotation  * Swerve.MAX_SPEED : 0;

        /*driveY = Math.pow(driveY,2) * Math.signum(driveY);
        driveX = Math.pow(driveX,2) * Math.signum(driveX);
        rotation = Math.pow(rotation,2) * Math.signum(rotation);
         */

        boolean isFiledRelative = SmartDashboard.getBoolean("Is Field Relative?", false);
        this.swerve.drive(driveY /2 ,driveX / 2 ,rotation/2, true );
    }

    @Override
    public void end(FinishReason reason) {
        swerve.stop();
    }
}
