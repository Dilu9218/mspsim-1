package se.sics.mspsim.platform.sky;

import java.io.IOException;
import se.sics.mspsim.chip.BackscatterTagRadio;
//import se.sics.mspsim.chip.CC2420;
import se.sics.mspsim.chip.FileStorage;
import se.sics.mspsim.chip.M25P80;
import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.core.USART;
import se.sics.mspsim.core.USARTSource;

public class BackscatterTagNode extends SkyNode {
    
    public BackscatterTagRadio tag;
    
    public BackscatterTagNode() {
        super();
/**/    System.out.println("BackscatterTagNode");
    }
    
    
    //USARTListener
    @Override
    public void dataReceived(USARTSource source, int data) {
/**/    System.out.println("BackscatterTagNode.dataReceived");
        if(radio.getChipSelect()) {
            radio.dataReceived(source, data);  
        } else {
            tag.dataReceived(source, data);
        }
        flash.dataReceived(source, data);
        /* if nothing selected, just write back a random byte to these devs */
        if (!radio.getChipSelect() && !flash.getChipSelect()) {
/**/        System.out.println("flash.getChipSelect: " + flash.getChipSelect());
/**/        System.out.println("radio.getChipSelect: " + radio.getChipSelect());
            source.byteReceived(0);
        }
    }
    
    @Override
    public void setupNodePorts() {
        super.setupNodePorts();
/**/    System.out.println("1.BackscatterTagNode.setupNodePorts");

        tag = new BackscatterTagRadio();  
        
/**/    System.out.println("2.BackscatterTagNode.setupNodePorts");
    
        if (getFlash() == null) {
            setFlash(new M25P80(cpu));
        }
        if (flashFile != null) {
            getFlash().setStorage(new FileStorage(flashFile));
        }
      }
    
}
