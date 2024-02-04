package frc.robot.subsystems;

import com.flash3388.flashlib.scheduling.Subsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends Subsystem {
    private AddressableLED addressableLED;
    private AddressableLEDBuffer addBuf;
    int length = 30;
    public LEDs() {
        this.addressableLED = new AddressableLED(9);
        this.addBuf = new AddressableLEDBuffer(length);
        this.addressableLED.setLength(length);
        addressableLED.setData(addBuf);
        addressableLED.start();
    }

    public void setColor(int hue, int saturation, int value ){
     for(int i =0; i < this.addBuf.getLength(); i++){
         this.addBuf.setHSV(i, hue, saturation, value);
     }
     this.addressableLED.setData(this.addBuf);
    }
    public void setColorRed(){
        for(var i =0; i < this.addBuf.getLength(); i++){
            this.addBuf.setHSV(i, 0, 100, 100);
        }
        this.addressableLED.setData(this.addBuf);
    }



}
