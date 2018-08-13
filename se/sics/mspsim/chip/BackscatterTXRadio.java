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

import se.sics.mspsim.chip.CC2420.RadioState;
import se.sics.mspsim.core.Chip;
import se.sics.mspsim.core.TimeEvent;
import se.sics.mspsim.core.USARTListener;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.core.MSP430Core;
import se.sics.mspsim.util.ArrayFIFO;
import se.sics.mspsim.util.CCITT_CRC;
import se.sics.mspsim.util.Utils;

public class BackscatterTXRadio extends Chip implements USARTListener, RFSource {
	
  // The Operation modes of the backscatter tag
  public static final int MODE_TXRX_OFF = 0x00;
  public static final int MODE_RX_ON = 0x01;
  public static final int MODE_TX_ON = 0x02;
  private static final String[] MODE_NAMES = new String[] {
   "off", "listen", "transmit"
  };

  // State Machine
  public enum RadioState {
     IDLE(1),
     RX_SFD_SEARCH(3),
     RX_FRAME(16),
     RX_DATA_TRANSFER(18),
     TX_DATA_TRANSFER(33),
     TX_FRAME(37);

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
  
  private RadioState stateMachine = RadioState.IDLE;
  private UartState state = UartState.DATA_WAIT;

  private ArrayFIFO rxFIFO;
  // More than needed...
  private int[] memory = new int[512];
  
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
  
  private USARTSource uart = null;
  private boolean UART_send_high_next = true;
  private int UART_current_byte;

  private CCITT_CRC txCrc = new CCITT_CRC();

  // 802.15.4 symbol period in ms
  public static final double SYMBOL_PERIOD = 0.016; // 16 us
  
  public static final double UART_BYTE_DURATION = 0.1; // 100 us
  
  public BackscatterTXRadio(MSP430Core cpu) {
    super("BackscatterTXRadio", cpu);
    rxFIFO = new ArrayFIFO("RXFIFO", memory, 0, 128);
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

    rxFIFO.reset();
  }

  public TimeEvent sendEvent = new TimeEvent(0, "BackscatterTag Send") {
    public void execute(long t) {
      txNext();
    }
  };
  
  public TimeEvent uartSendEvent = new TimeEvent(0, "BackscatterTag UART Send") {
	@Override
	public void execute(long t) {
		uartTxNext();
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
    	if (uart != null) {
    		uart.byteReceived('k');
    	}
    	setState(RadioState.TX_DATA_TRANSFER);
    }
  }
  
  private void uartTxNext() {
	  if (!rxFIFO.isEmpty()) {
		  Byte tx_byte;
		  if (UART_send_high_next) {
			  UART_current_byte = rxFIFO.read();
			  tx_byte = (byte) Integer.toHexString((UART_current_byte & 0xFF) >> 4).charAt(0);
		  } else {
			  tx_byte = (byte) Integer.toHexString((UART_current_byte & 0x0F)).charAt(0);
		  }
		  System.out.println("Current_byte: " + UART_current_byte + "tx_byte: " + tx_byte);
		  uart.byteReceived(tx_byte);
		  cpu.scheduleTimeEventMillis(uartSendEvent, UART_BYTE_DURATION);
		  UART_send_high_next = !UART_send_high_next;
	  } else {
		  uart.byteReceived('\n');
//		  uart = null;
		  rxFIFO.reset();
		  setState(RadioState.RX_SFD_SEARCH);
	  }
  }
  
  private boolean setState(RadioState new_state) {
	  stateMachine = new_state;
	  System.out.println("Setting state: " + new_state);
	  
	  switch (new_state) {
	  case IDLE:
		  break;
	  case RX_SFD_SEARCH:
		  zeroSymbols = 0;
		  rxFIFO.reset();
		  break;
		  
	  case RX_FRAME:
		  rxlen = 0;
		  rxread = 0;
		  break;
		  
	  case RX_DATA_TRANSFER:
		  UART_send_high_next = true;
		  state = UartState.DATA_WAIT;
		  break;
		 
	  case TX_DATA_TRANSFER:
		  rxFIFO.reset();
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
	  uart = source;
	  System.out.println("Data received " + data);
	  if (state == UartState.DATA_WAIT) {
		  switch (stateMachine) {
		  	case IDLE:
				if ((data & 0xFF) == '\n') {
					switch (last_char) {
						case 'r':
						case 'R':
							setState(RadioState.RX_SFD_SEARCH);
							break;
						case 's':
						case 'S':
							setState(RadioState.TX_DATA_TRANSFER);
							break;
						case 'q':
						case 'Q':
							source.byteReceived(0);
							break;
						case 'o':
						case 'O':
							setState(RadioState.IDLE);
							break;
							
						default:
							break;
					}
					last_char = '\0';
				} else {
					last_char = (char)(data & 0xFF);
				}
		  		break;
			case TX_DATA_TRANSFER:
				if ((data & 0xFF) == '\n') {
					switch (last_char) {
						case 'r':
						case 'R':
							setState(RadioState.RX_SFD_SEARCH);
							break;
						case 'q':
						case 'Q':
							source.byteReceived(0);
							break;
						case 'o':
						case 'O':
							setState(RadioState.IDLE);
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
//						        rxFIFO.read(); // Drop the length byte from the FIFO
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
						case 'q':
						case 'Q':
							source.byteReceived(0);
							break;
						case 'o':
						case 'O':
							setState(RadioState.IDLE);
							break;
							
						default:
							break;
					}
					last_char = '\0';
				} else {
					last_char = (char)(data & 0xFF);
				}
				break;
				
			case RX_DATA_TRANSFER:
				if ((data & 0xFF) == '\n') {
					switch (last_char) {
						case 's':
						case 'S':
							setState(RadioState.TX_DATA_TRANSFER);
							break;
						case 'q':
						case 'Q':
							if (rxFIFO.isEmpty()) {
								source.byteReceived(0);
							} else {
//								uart = source;
								uartTxNext();
							}
							break;
						case 'o':
						case 'O':
							setState(RadioState.IDLE);
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
  public int getMode() {
	  if (stateMachine == RadioState.IDLE) {
		  return MODE_TXRX_OFF;
	  } else if (stateMachine == RadioState.RX_DATA_TRANSFER ||
			  stateMachine == RadioState.RX_FRAME || 
			  stateMachine == RadioState.RX_SFD_SEARCH) {
		  return MODE_RX_ON;
	  } else {
		  return MODE_TX_ON;
	  }
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
              //              if (logLevel > INFO) log("RX: Preamble/SFD Synchronized.");
              setState(RadioState.RX_FRAME);
          } else {
              /* if not four zeros and 0x7A then no zeroes... */
              zeroSymbols = 0;
          }

      } else if(stateMachine == RadioState.RX_FRAME) {
         if(rxFIFO.isFull()) {
              rxFIFO.reset();
              setState(RadioState.RX_SFD_SEARCH);
          } else {
                  rxFIFO.write(data);
                  if (rxread == 0) {
                      rxlen = data & 0xff;
                      rxFIFO.read();
                  }
              if (rxread++ == rxlen) {
            	  setState(RadioState.RX_DATA_TRANSFER);
            	  uartTxNext();
              }
         }
      }
  }

} /* BackscatterTXRadio */
