package frc.robot.Actions;

import com.flash3388.flashlib.scheduling.ActionControl;
import com.flash3388.flashlib.scheduling.FinishReason;
import com.flash3388.flashlib.scheduling.actions.ActionBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ShooterSystem;

public class ForwardShooter extends ActionBase {
    private ShooterSystem shooter;

    public ForwardShooter(ShooterSystem shooter){
        this.shooter = shooter;
        requires(shooter);

        configure().setName("ForwardShooter").save();
    }

    @Override
    public void initialize(ActionControl control) {
        shooter.resetPID();
    }

    @Override
    public void execute(ActionControl actionControl) {
        SmartDashboard.putBoolean("ForwardBool", true);
        shooter.shoot();
    }

    @Override
    public void end(FinishReason reason) {
        SmartDashboard.putBoolean("hgjkglkjhjkhjkhlhjlkhjlkhkljhjlkj",true);
        //new StopWheels(shooter).start();




    }
}
