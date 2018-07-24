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
import java.util.Arrays; //can remove this later

import se.sics.mspsim.chip.CC2420.RadioState;
import se.sics.mspsim.core.Chip;
import se.sics.mspsim.core.TimeEvent;
import se.sics.mspsim.core.USARTListener;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.core.MSP430Core;
import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.util.ArrayFIFO;
import se.sics.mspsim.util.CCITT_CRC;
import se.sics.mspsim.util.Utils;

public class BackscatterTXRadio extends Chip implements USARTListener, RFSource {
	
  // The Operation modes of the backscatter tag
  public static final int MODE_RX_ON = 0x00;
  public static final int MODE_TX_ON = 0x01;
  private static final String[] MODE_NAMES = new String[] {
   "listen", "transmit"
  };

  // State Machine
  public enum RadioState {
     RX_SFD_SEARCH(3),
     RX_FRAME(16),
     RX_OVERFLOW(17),
     RX_NOTIFY(18),
     TX_DATA_TRANSFER(33),
     TX_FRAME(37),
     TX_UNDERFLOW(56);

     private final int state;
     RadioState(int stateNo) {
       state = stateNo;
     }

     public int getFSMState() {
       return state;
     }
  };

  public enum UartState {
    DATA_WAIT,
    TX_FRAME_WAIT
  }

  private RFListener rfListener;
  
  private RadioState stateMachine = RadioState.TX_DATA_TRANSFER;
  private UartState state = UartState.DATA_WAIT;

  private ArrayFIFO rxFIFO;
  
  private int[] txBuffer;
  private int txbufferPos = 6;
  private int bufferPos = 0;
  private int payload = 0;
  private int payloadLength = 0;
  private char last_char = '\0';
  private int char_count = 0;
  private int zeroSymbols = 0;
  private int rxlen = 0;
  private int rxread = 0;
  private boolean overflow = false;

  private boolean shouldAck = false;
  private boolean currentSFD;
  private boolean currentFIFO;
  private boolean currentFIFOP;

  public static final int REG_IOCFG0        = 0x1C;
  public static final int FIFO_POLARITY = (1<<10);
  public static final int FIFOP_POLARITY = (1<<9);
  public static final int SFD_POLARITY = (1<<8);
  public static final int RAM_RXFIFO    = 0x080;
  private int[] memory = new int[512];
  private int[] registers = new int[64];

  private IOPort fifopPort = null;
  private int fifopPin;

  private IOPort fifoPort = null;
  private int fifoPin;

  private IOPort sfdPort = null;
  private int sfdPin;

  private CCITT_CRC txCrc = new CCITT_CRC();

  // 802.15.4 symbol period in ms
  public static final double SYMBOL_PERIOD = 0.016; // 16 us\
  
  public BackscatterTXRadio(MSP430Core cpu) {
    super("BackscatterTXRadio", cpu);
    rxFIFO = new ArrayFIFO("RXFIFO", memory, RAM_RXFIFO, 128);
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

    currentFIFOP = false;
    rxFIFO.reset();
    overflow = false;
  }

  public TimeEvent sendEvent = new TimeEvent(0, "BackscatterTag Send") {
    public void execute(long t) {
      txNext();
    }
  };

  private void txNext() {
	  stateMachine = RadioState.TX_FRAME;

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
    	setState(RadioState.TX_DATA_TRANSFER);
    }
  }
  
  private boolean setState(RadioState new_state) {
	  stateMachine = new_state;
	  
	  switch (new_state) {
	  case RX_SFD_SEARCH:
		  zeroSymbols = 0;
		  break;
		  
	  case RX_FRAME:
		  rxlen = 0;
		  rxread = 0;
		  break;
		 
	  case TX_DATA_TRANSFER:
		  txbufferPos = 6;
		  bufferPos = 0;
		  payload = 0;
		  payloadLength = 0;
		  last_char = '\0';
		  char_count = 0;
		  state = UartState.DATA_WAIT;
		  break;
		  
	  default:
		  break;
	  }
	  return true;
  }

  public void dataReceived(USARTSource source, int data) {
	  if (state == UartState.DATA_WAIT) {
		  switch (stateMachine) {
			case TX_DATA_TRANSFER:
				if ((data & 0xFF) == '\n') {
					switch (last_char) {
						case 'r':
						case 'R':
							setState(RadioState.RX_SFD_SEARCH);
							break;
						case 's':
						case 'S':
							break;
							
						default:
							if (payload > 0) {
								// The payloadLegth consists of the payload itself plus 2 bytes
						        // for the CRC.
						        payloadLength =  payload + 2;
						        // txBuffer[5] contains the length of the PSDU(p.36 - CC2420 datasheet). 
						        txBuffer[5] = (payloadLength & 0x7F);
						        
						        payload = 0;
						        txbufferPos = 6;
						        
						        /* Transmit */
						        txNext();
						        state = UartState.TX_FRAME_WAIT;
							}
					}
					last_char = '\0';
				} else {
					last_char = (char)(data & 0xFF);
					int digit = Character.digit(last_char, 16);
					if (digit != -1) { // It's a valid Hex character. Put in buffer
						char_count++;
						if (char_count % 2 == 0) {
							txBuffer[txbufferPos] |= (digit & 0x0F);
							txbufferPos++;
							payload++;
						} else {
							txBuffer[txbufferPos] = digit << 4;
						}
						last_char = '\0';
					} 
				}
				break;
			
			case RX_SFD_SEARCH:
				if ((data & 0xFF) == '\n') {
					switch (last_char) {
						case 's':
						case 'S':
							setState(RadioState.TX_DATA_TRANSFER);
							break;
						default:
							break;
					}
					last_char = '\0';
				} else {
					last_char = (char)(data & 0xFF);
				}
				break;
		
			default:
				break;
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
  
  /* Receive a byte from the radio medium
   * @see se.sics.mspsim.chip.RFListener#receivedByte(byte)
   */
  public void receivedByte(byte data) {
//	  System.out.println("RX BYTE: " + data + " State: " + stateMachine);
      
      // Received a byte from the "air"
//      if (logLevel > INFO)
//        log("RF Byte received: " + Utils.hex8(data) + " state: " + stateMachine + " noZeroes: " + zeroSymbols +
//              ((stateMachine == RadioState.RX_SFD_SEARCH || stateMachine == RadioState.RX_FRAME) ? "" : " *** Ignored"));
//
      if(stateMachine == RadioState.RX_SFD_SEARCH) {
          // Look for the preamble (4 zero bytes) followed by the SFD byte 0x7A
          if(data == 0) {
              // Count zero bytes
              zeroSymbols++;
          } else if(zeroSymbols >= 4 && data == 0x7A) {
              // If the received byte is !zero, we have counted 4 zero bytes prior to this one,
              // and the current received byte == 0x7A (SFD), we're in sync.
              // In RX mode, SFD goes high when the SFD is received
//              setSFD(true);
//              if (logLevel > INFO) log("RX: Preamble/SFD Synchronized.");
              setState(RadioState.RX_FRAME);
          } else {
              /* if not four zeros and 0x7A then no zeroes... */
              zeroSymbols = 0;
          }

      } else if(stateMachine == RadioState.RX_FRAME) {
          System.out.println("rxfifo---lenght-before--overflow-------------------------------");
          System.out.println(rxFIFO.length());
          if (overflow) {
              System.out.println("----------buf overflow--------------------------");
              /* if the CC2420 RX FIFO is in overflow - it needs a flush before receiving again */
          } else if(rxFIFO.isFull()) {
              System.out.println("rxfifo----before--overflow-------------------------------");
              setRxOverflow();
              System.out.println("rxfifo---overflow-------------------------------");
          } else {
              System.out.println("no problem------------------------------");
                  rxFIFO.write(data);
                  if (rxread == 0) {
//                      rxCrc.setCRC(0);
                      rxlen = data & 0xff;
                      System.out.println("Starting to get packet. len = " + rxlen);
//                      decodeAddress = addressDecode;
//                      if (logLevel > INFO) log("RX: Start frame length " + rxlen);
//                      // FIFO pin goes high after length byte is written to RXFIFO
//                      setFIFO(true);
                  } //else if (rxread < rxlen - 1) {
//                      /* As long as we are not in the length or FCF (CRC) we count CRC */
//                      rxCrc.addBitrev(data & 0xff);
//                      if (rxread == 1) {
//                          fcf0 = data & 0xff;
//                          frameType = fcf0 & FRAME_TYPE;
//                      } else if (rxread == 2) {
//                          fcf1 = data & 0xff;
//                          if (frameType == TYPE_DATA_FRAME || frameType == TYPE_CMD_FRAME) {
//                              ackRequest = (fcf0 & ACK_REQUEST) > 0;
//                              destinationAddressMode = (fcf1 >> 2) & 3;
//                              /* check this !!! */
//                              if (addressDecode && destinationAddressMode != LONG_ADDRESS &&
//                                      destinationAddressMode != SHORT_ADDRESS) {
//                                  rejectFrame();
//                              }
//                          } else if (frameType == TYPE_BEACON_FRAME ||
//                                  frameType == TYPE_ACK_FRAME){
//                              decodeAddress = false;
//                              ackRequest = false;
//                          } else if (addressDecode) {
//                              /* illegal frame when decoding address... */
//                              rejectFrame();
//                          }
//                      } else if (rxread == 3) {
//                          // save data sequence number
//                          dsn = data & 0xff;
//                      } else if (decodeAddress) {
//                          boolean flushPacket = false;
//                          /* here we decode the address !!! */
//                          if (destinationAddressMode == LONG_ADDRESS && rxread == 8 + 5) {
//                              /* here we need to check that this address is correct compared to the stored address */
//                              flushPacket = !rxFIFO.tailEquals(memory, RAM_IEEEADDR, 8);
//                              flushPacket |= !rxFIFO.tailEquals(memory, RAM_PANID, 2, 8)
//                                      && !rxFIFO.tailEquals(BC_ADDRESS, 0, 2, 8);
//                              decodeAddress = false;
//                          } else if (destinationAddressMode == SHORT_ADDRESS && rxread == 2 + 5){
//                              /* should check short address */
//                              flushPacket = !rxFIFO.tailEquals(BC_ADDRESS, 0, 2)
//                                      && !rxFIFO.tailEquals(memory, RAM_SHORTADDR, 2);
//                              flushPacket |= !rxFIFO.tailEquals(memory, RAM_PANID, 2, 2)
//                                      && !rxFIFO.tailEquals(BC_ADDRESS, 0, 2, 2);
//                              decodeAddress = false;
//                          }
//                          if (flushPacket) {
//                              rejectFrame();
//                          }
//                      }
//                  }
//
//                  /* In RX mode, FIFOP goes high when the size of the first enqueued packet exceeds
//                   * the programmable threshold and address recognition isn't ongoing */ 
//                  if (currentFIFOP == false
//                          && rxFIFO.length() <= rxlen + 1
//                          && !decodeAddress && !frameRejected
//                          && rxFIFO.length() > fifopThr) {
//                      setFIFOP(true);
//                      if (logLevel > INFO) log("RX: FIFOP Threshold reached - setting FIFOP");
//                  }
//              }
//
              if (rxread++ == rxlen) {
//                  if (frameRejected) {
//                      if (logLevel > INFO) log("Frame rejected - setting SFD to false and RXWAIT\n");
//                      setSFD(false);
//                      setState(RadioState.RX_WAIT);
//                      return;
//                  }
//                  // In RX mode, FIFOP goes high, if threshold is higher than frame length....
//
//                  // Here we check the CRC of the packet!
//                  //System.out.println("Reading from " + ((rxfifoWritePos + 128 - 2) & 127));
//                  int crc = rxFIFO.get(-2) << 8;
//                  crc += rxFIFO.get(-1); //memory[RAM_RXFIFO + ((rxfifoWritePos + 128 - 1) & 127)];
//
//                  crcOk = crc == rxCrc.getCRCBitrev();
//                  if (logLevel > INFO && !crcOk) {
//                      log("CRC not OK: recv:" + Utils.hex16(crc) + " calc: " + Utils.hex16(rxCrc.getCRCBitrev()));
//                  }
//                  // Should take a RSSI value as input or use a set-RSSI value...
//                  rxFIFO.set(-2, registers[REG_RSSI] & 0xff); 
//                  rxFIFO.set(-1, (corrval & 0x7F) | (crcOk ? 0x80 : 0));
//                  //          memory[RAM_RXFIFO + ((rxfifoWritePos + 128 - 2) & 127)] = ;
//                  //          // Set CRC ok and add a correlation - TODO: fix better correlation value!!!
//                  //          memory[RAM_RXFIFO + ((rxfifoWritePos + 128 - 1) & 127)] = 37 |
//                  //              (crcOk ? 0x80 : 0);
//
//                  /* set FIFOP only if this is the first received packet - e.g. if rxfifoLen is at most rxlen + 1
//                   * TODO: check what happens when rxfifoLen < rxlen - e.g we have been reading before FIFOP */
//                  if (rxFIFO.length() <= rxlen + 1) {
//                      setFIFOP(true);
//                  } else {
//                      if (logLevel > INFO) log("Did not set FIFOP rxfifoLen: " + rxFIFO.length() + " rxlen: " + rxlen);
//                  }
//                  setSFD(false);
//                  if (logLevel > INFO) log("RX: Complete: packetStart: " + rxFIFO.stateToString());
//
//                  /* if either manual ack request (shouldAck) or autoack + ACK_REQ on package do ack! */
//                  /* Autoack-mode + good CRC => autoack */
//                  if (((autoAck && ackRequest) || shouldAck) && crcOk) {
//                      setState(RadioState.TX_ACK_CALIBRATE);
//                  } else {
//                      setState(RadioState.RX_WAIT);
//                  }
            	  setState(RadioState.RX_SFD_SEARCH);
              }
             }
          }
//      }
  }

    private void setSFD(boolean sfd) {
        currentSFD = sfd;
        if( (registers[REG_IOCFG0] & SFD_POLARITY) == SFD_POLARITY)
            sfdPort.setPinState(sfdPin, sfd ? IOPort.PinState.LOW : IOPort.PinState.HI);
        else
            sfdPort.setPinState(sfdPin, sfd ? IOPort.PinState.HI : IOPort.PinState.LOW);
        if (logLevel > INFO) log("SFD: " + sfd + "  " + cpu.cycles);
    }
    private void setFIFOP(boolean fifop) {
        currentFIFOP = fifop;
        if (logLevel > INFO) log("Setting FIFOP to " + fifop);
        if( (registers[REG_IOCFG0] & FIFOP_POLARITY) == FIFOP_POLARITY) {
            fifopPort.setPinState(fifopPin, fifop ? IOPort.PinState.LOW : IOPort.PinState.HI);
        } else {
            fifopPort.setPinState(fifopPin, fifop ? IOPort.PinState.HI : IOPort.PinState.LOW);
        }
    }

    private void setFIFO(boolean fifo) {
        currentFIFO = fifo;
        if (logLevel > INFO) log("Setting FIFO to " + fifo);
        if((registers[REG_IOCFG0] & FIFO_POLARITY) == FIFO_POLARITY) {
            fifoPort.setPinState(fifoPin, fifo ? IOPort.PinState.LOW : IOPort.PinState.HI);
        } else {
            fifoPort.setPinState(fifoPin, fifo ? IOPort.PinState.HI : IOPort.PinState.LOW);
        }
    }

    private void setRxOverflow() {
        if (logLevel > INFO) log("RXFIFO Overflow! Read Pos: " + rxFIFO.stateToString());
        setFIFOP(true);
        setFIFO(false);
        setSFD(false);
        overflow = true;
        shouldAck = false;
        setState(RadioState.RX_OVERFLOW);
    }

    public void setFIFOPPort(IOPort port, int pin) {
        fifopPort = port;
        fifopPin = pin;
    }

    public void setFIFOPort(IOPort port, int pin) {
        fifoPort = port;
        fifoPin = pin;
    }

    public void setSFDPort(IOPort port, int pin) {
        sfdPort = port;
        sfdPin = pin;
    }
} /* BackscatterTXRadio */
