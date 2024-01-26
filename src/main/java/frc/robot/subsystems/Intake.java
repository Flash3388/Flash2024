package frc.robot.subsystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.revrobotics.CANSparkMax;

public class Intake extends Subsystem {
    private CANSparkMax motor;
    private final static double SPEED = -0.2;

    public Intake(CANSparkMax motor){
        this.motor = motor;
    }

    public void stop(){
        this.motor.stopMotor();
    }

    public void takeIn(){
        this.motor.set(SPEED);
    }

    public void takeOut(){
        this.motor.set(-SPEED);
    }
}
