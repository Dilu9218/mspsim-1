package se.sics.mspsim.platform.sky;

import se.sics.mspsim.chip.BackscatterTXRadio;
import se.sics.mspsim.core.USARTSource;

public class BackscatterTagNode extends SkyNode {
    
  public BackscatterTXRadio tag;
    
  public BackscatterTagNode() {
      super();
  }
    
  //USARTListener
  @Override
  public void dataReceived(USARTSource source, int data) {
    boolean tagSelected = false;
      
    /* 
     * In case a future developer would also like to use the CC2420 radio chip.
     * For the use of our Backscatter tag, CC2420 is not selected and USART0 
     * is used this time for the serial communication between the Mspsim 
     * and the Backscatter tag.
     */
    
    if(radio.getChipSelect()) {
      /* CC2420 in not selected whenever in Contiki code UART0 "sends" to the tag */
      radio.dataReceived(source, data);  
    } else {
      tagSelected = true;
      tag.dataReceived(source, data);
    }
    flash.dataReceived(source, data);
    /* if nothing selected, just write back a random byte to these devs */
    if (!radio.getChipSelect() && !tagSelected && !flash.getChipSelect()) {
        source.byteReceived(0);
    }
  }
    
  @Override
  public void setupNodePorts() {
    super.setupNodePorts();
      
    /* Creation of the Backscatter TX Radio module */
    tag = new BackscatterTXRadio(cpu);
  }
  
    
}