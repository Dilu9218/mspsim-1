package se.sics.mspsim.platform.sky;
import java.io.IOException;
import se.sics.mspsim.chip.FileStorage;
import se.sics.mspsim.chip.M25P80;
import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.util.ArgumentManager;

/**
 * Emulation of Sky Mote
 */
public class TagNode extends MoteIVNode {

  private M25P80 flash;

  /**
   * Creates a new <code>TagNode</code> instance.
   *
   */
  public TagNode() {
    super("Tmote Sky");
  }

  public M25P80 getFlash() {
    return flash;
  }

  public void setFlash(M25P80 flash) {
    this.flash = flash;
    registry.registerComponent("xmem", flash);
  }

  // USART Listener
  public void dataReceived(USARTSource source, int data) {
    radio.dataReceived(source, data);
    flash.dataReceived(source, data);
    /* if nothing selected, just write back a random byte to these devs */
    if (!radio.getChipSelect() && !flash.getChipSelect()) {
      source.byteReceived(0);
    }
  }

  @Override
  protected void flashWrite(IOPort source, int data) {
    flash.portWrite(source, data);
  }
  
  public void setupNodePorts() {
    super.setupNodePorts();
    if (getFlash() == null) {
        setFlash(new M25P80(cpu));
    }
    if (flashFile != null) {
        getFlash().setStorage(new FileStorage(flashFile));
    }
  }

  public static void main(String[] args) throws IOException {
    TagNode node = new TagNode();
    ArgumentManager config = new ArgumentManager();
    config.handleArguments(args);
    node.setupArgs(config);
  }

}
