package se.sics.mspsim.platform.sky;

import se.sics.mspsim.chip.BackscatterTXRadio;
import se.sics.mspsim.core.USARTSource;

public class BackscatterTagNode extends SkyNode {
    
    public BackscatterTXRadio tag;
    
    public BackscatterTagNode() {
        super();
/**/    System.out.println("BackscatterTagNode");
    }
    
    //USARTListener
    @Override
    public void dataReceived(USARTSource source, int data) {
        boolean tagSelected = false;
      
/**/    //System.out.println(tag.hashCode() + " BackscatterTagNode.dataReceived");
        //super.dataReceived(source, data);
        // In case a future developer would also like to use the CC2420 radio chip.
        // For the use of our Backscatter tag, CC2420 is not selected and USART0 
        // is used this time for the serial communication between the Mspsim 
        // and the Backscatter tag.
        if(radio.getChipSelect()) {
          /* CC2420 in not selected whenever in Contiki code UART0 "sends" to the tag */
/**/      System.out.println(radio.hashCode() + " radio.dataReceived");            
          radio.dataReceived(source, data);  
        } else {
/**/      System.out.println("1. - " + tag.hashCode() + " tag.dataReceived");          
          tagSelected = true;
          tag.dataReceived(source, data);
/**/      System.out.println("2. - " + tag.hashCode() + " tag.dataReceived");

        }
        flash.dataReceived(source, data);
        /* if nothing selected, just write back a random byte to these devs */
        if (!radio.getChipSelect() && !tagSelected && !flash.getChipSelect()) {
/**/        System.out.println(tag.hashCode() + " flash.getChipSelect: " + flash.getChipSelect());
/**/        System.out.println(tag.hashCode() + " radio.getChipSelect: " + radio.getChipSelect());
            source.byteReceived(0);
        }
    }
    
    @Override
    public void setupNodePorts() {
        super.setupNodePorts();
/**/    System.out.println("1.BackscatterTagNode.setupNodePorts");
        
        /* Creation of the Backscatter TX Radio module */
        tag = new BackscatterTXRadio(cpu);
/**/    System.out.println("2 " + tag.hashCode() + " BackscatterTagNode.setupNodePorts");
    }
}
