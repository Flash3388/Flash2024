package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Intake;

public class SystemFactory {
    public static Intake createIntake(){
    CANSparkMax canSparkMax = new CANSparkMax(RobotMap.INTAKE, CANSparkLowLevel.MotorType.kBrushless);
    Intake intake = new Intake(canSparkMax, new DigitalInput(6));
    return intake;
    }

    public static AddressableLED createAddressableLED(){
        int length = 30;
        AddressableLED addressableLED = new AddressableLED(9);
        addressableLED.setLength(length);
        AddressableLEDBuffer addBuf = new AddressableLEDBuffer(length);
        addressableLED.setData(addBuf);
        addressableLED.start();
        return addressableLED;
    }
}
