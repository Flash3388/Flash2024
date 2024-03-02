package frc.robot.actions;

import com.flash3388.flashlib.hid.XboxController;
import com.flash3388.flashlib.robot.RunningRobot;
import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import com.flash3388.flashlib.time.Clock;
import com.flash3388.flashlib.time.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subSystems.Arm;
import frc.robot.subSystems.Intake;
import frc.robot.subSystems.Limelight;
import frc.robot.subSystems.ShooterSystem;

public class SetPointAngleByVision extends ActionBase {

    private Limelight limelight;
    private Intake intake;
    private Arm arm;
    private ShooterSystem shooterSystem;


    public SetPointAngleByVision(Limelight limelight, Intake intake, Arm arm, ShooterSystem shooter){
        this.intake = intake;
        this.limelight = limelight;
        this.arm = arm;
        this.shooterSystem = shooter;

        //configure().setName("SetPointAngleByVision").save();

        requires(shooter);
    }

    @Override
    public void initialize(ActionControl control) {
        arm.baseOnLimelightDetection();
        limelight.startTimer();
        Limelight.KEEP_UPDATING_ODOMETER = false;
    }

    @Override
    public void execute(ActionControl control) {
        if(intake.isIN() && arm.isBasedOnLimelightDetection()) {
            double distance = limelight.getDisHorizontalToTarget();

            SmartDashboard.putNumber("odometer distance", distance);
             double k = SmartDashboard.getNumber("k of angle", 19.5);

            //double angle = -1.05 * Math.pow(distance, 2) + 11.2 * distance + 19.5 ; //18.4
            double angle = -1.05 * Math.pow(distance, 2) + 11.2 * distance + k ; //18.4
            //double angle = -1.82 * Math.pow(distance, 2) + 15.5 * distance + 12.1;
            arm.setSetPointAngle(angle);
            shooterSystem.shootSpeaker();
        }
        else control.finish();
    }

    @Override
    public void end(FinishReason reason) {
        arm.doNotBaseOnLimelightDetection(); //to be sure
        limelight.stopTimer();
        shooterSystem.moveDefault(intake.isIN());
        Limelight.KEEP_UPDATING_ODOMETER = true;
    }
}
