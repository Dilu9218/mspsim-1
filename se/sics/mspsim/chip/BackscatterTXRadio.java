/**
 * Copyright (c) 2007-2012 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of MSPSim.
 *
 * -----------------------------------------------------------------
 *
 * CC2420
 *
 * Author  : Joakim Eriksson
 * Created : Sun Oct 21 22:00:00 2007
 *
 */

package se.sics.mspsim.chip;

import se.sics.mspsim.core.EmulationLogger.WarningType;

import java.util.ArrayList;

import se.sics.mspsim.core.Chip;
import se.sics.mspsim.core.TimeEvent;
import se.sics.mspsim.core.USARTListener;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.core.MSP430Core;
import se.sics.mspsim.util.CCITT_CRC;

public class BackscatterTXRadio extends Chip implements USARTListener, RFSource {

  public enum UartState {
    FIRST_CHAR, SECOND_CHAR
  }

  private RFListener rfListener;
  private UartState state = UartState.FIRST_CHAR;

  private int[] txBuffer;
  private int txbufferPos = 6;
  private int bufferPos = 0;
  private int dataValue = 0;
  private int payload = 0;
  private int payloadLength = 0;
  private int intermediateValue = 0;
  private boolean startFillingTxBuffer = false;
  private boolean nextChar = false;
  private boolean ongoingTransmsission = false;

  private CCITT_CRC txCrc = new CCITT_CRC();

  // 802.15.4 symbol period in ms
  public static final double SYMBOL_PERIOD = 0.016; // 16 us\
  
  public BackscatterTXRadio(MSP430Core cpu) {
    super("BackscatterTXRadio", cpu);

    /*
     * page 36, CC2420 datasheet
     * 
     * Format of the 802.15.4 packet: Preamble: 4 bytes SFD: 1 byte Length: 1
     * byte Payload(PSDU): 127 bytes including FCS Packet length: 133 bytes
     */

    txBuffer = new int[133];
    /* Set the preamble for protocol 802.15.4. */
    txBuffer[0] = 0;
    txBuffer[1] = 0;
    txBuffer[2] = 0;
    txBuffer[3] = 0;
    /* Set the sfd for protocol 802.15.4. */
    txBuffer[4] = 0x7A;
    /* Contains the length of the PPDU. */
    txBuffer[5] = 0;
  }

  public TimeEvent sendEvent = new TimeEvent(0, "BackscatterTag Send") {
    public void execute(long t) {
      txNext();
    }
  };

  private void txNext() {
    ongoingTransmsission = true;

    // The length of the packet consists of the length of the payload the has
    // been calculated and the first 6 bytes which constitute the synchronization
    // header.
    int packetLength = 6 + payloadLength;
    if (bufferPos < packetLength) {
      
      /* Calculation of the CRC */
      if (bufferPos == packetLength - 2) {
        txCrc.setCRC(0);
        for (int i = 6; i < packetLength - 2; i++) {
          txCrc.addBitrev(txBuffer[i]);
        }
        txBuffer[packetLength - 2] = txCrc.getCRCHi();
        txBuffer[packetLength - 1] = txCrc.getCRCLow();
      }

      if (rfListener != null) {
        rfListener.receivedByte((byte) (txBuffer[bufferPos] & 0xFF));
      } 
      
      bufferPos++;
      // Two symbol periods to send a byte...
      cpu.scheduleTimeEventMillis(sendEvent, SYMBOL_PERIOD * 2);
    } else {
      bufferPos = 0;
      ongoingTransmsission = false;
    }
  }

  public void dataReceived(USARTSource source, int data) {

    // Send "s\n" before you start sending the packet through UART0.
    if ((data & 0xFF) == 's') {
      // If a packet is already being transmitted do no let 
      // another to be sent from the Mspsim.
      if(!ongoingTransmsission) {  
        nextChar = true;
      }
    } else if (((data & 0xFF) == '\n') && nextChar) {
      nextChar = false;
      startFillingTxBuffer = true;
      return;
    }
    
    /*
     * The value, written in hex, that Mspsim passes to our tag through uart0
     * can be represented in a binary notation of 4 bits. But UART sends each
     * value as a character occupying 8 bits. The following procedure is done in
     * order to combine two characters sent by UART in an actual meaningful byte
     * forming the 802.15.4 packet that our tag will finally transmit. This
     * procedure is done when UART sends a '\n' character.
     */

    // The following statement is executed 125 times for the calculation of the 
    // payload plus 1 for the detection of the new line character.
    if (startFillingTxBuffer && (payload < 126)) {
      if ((data & 0xFF) != '\n') {
        if ((data & 0xFF) <= 57) {
          // Subtracting 48 from an ascii number (0-9) gives you the
          // binary representation of that number.
          intermediateValue = (data & 0xFF) - 48;
        } else if ((data & 0xFF) <= 70) {
          // Subtracting 55 from an ascii upper case letter (A-F) gives
          // you the binary representation of that letter.
          intermediateValue = (data & 0xFF) - 55;
        } else if ((data & 0xFF) <= 102) {
          // Subtracting 55 from an ascii lower case letter(a-f) gives
          // you the binary representation of that letter.
          intermediateValue = (data & 0xFF) - 87;
        }

        switch (state) {
        case FIRST_CHAR:
          dataValue = intermediateValue << 4;
          state = UartState.SECOND_CHAR;
          break;

        case SECOND_CHAR:
          dataValue |= intermediateValue & 0x0F;
          txBuffer[txbufferPos++] = (dataValue & 0xFF);
          // Counts the bytes of the packet without including CRC
          payload++;
          state = UartState.FIRST_CHAR;
          break;
        }

      } else {
        // The payloadLegth consists of the payload itself plus 2 bytes
        // for the CRC.
        payloadLength =  payload + 2;
        // txBuffer[5] contains the length of the PSDU(p.36 - CC2420 datasheet). 
        txBuffer[5] = (payloadLength & 0x7F);
        
        payload = 0;
        txbufferPos = 6;
        startFillingTxBuffer = false;
        
        /* Transmit */
        txNext();
        
        state = UartState.FIRST_CHAR;
      }
    } else {
      // if the txBuffer is already full with the maximum effective payload raise a warning
      if (payload >= 126) {
        System.out.println("WARNING - PACKET SIZE TOO LARGE");
        logw(WarningType.EXECUTION, "Warning - packet size too large");
      } else if (ongoingTransmsission) {
//        System.out.println("Ongoing Transmission");
        logw(WarningType.EXECUTION, "Ongoing Transmission from the tag");
      } else if(!nextChar) {
        //logw(WarningType.EXECUTION, "No data sent by Mspsim yet!!");
//        System.out.println("No data sent by Mspsim");
      } else if (!startFillingTxBuffer) {
        //logw(WarningType.EXECUTION, "Character s came, waiting for new line character - No data sent by Mspsim yet!");
//        System.out.println("Character s came, waiting for new line character - No data sent by Mspsim yet!");
      }
    }
  }

  /* Not used by the tag */
  @Override
  public int getConfiguration(int parameter) {
    return 0;
  }

  /* Not used by the tag */
  @Override
  public int getModeMax() {
    return 0;
  }

  @Override
  public synchronized void addRFListener(RFListener rf) {
    rfListener = RFListener.Proxy.INSTANCE.add(rfListener, rf);
  }

  @Override
  public synchronized void removeRFListener(RFListener rf) {
    rfListener = RFListener.Proxy.INSTANCE.remove(rfListener, rf);
  }

} /* BackscatterTXRadio */
