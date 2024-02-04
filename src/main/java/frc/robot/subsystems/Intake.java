package frc.robot.subSystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends Subsystem {
    private CANSparkMax motor;
    private final static double SPEED = 0.5;
    private DigitalInput in;

    public Intake(CANSparkMax motor,  DigitalInput digitalInput){
        this.motor = motor;
        this.in = digitalInput;
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
    public boolean isIN(){ return !this.in.get();
    }
}
