/**
 * Copyright (c) 2018 Uppsala University
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
 * Author  : Carlos Perez Penichet
 *
 */

package se.sics.mspsim.chip;

import se.sics.mspsim.core.Chip;
import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.core.TimeEvent;
import se.sics.mspsim.core.USARTListener;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.core.MSP430Core;
import se.sics.mspsim.util.ArrayFIFO;
import se.sics.mspsim.util.CCITT_CRC;

public class BackscatterTXRadio extends Chip implements USARTListener, RFSource {
	private IOPort sfdPort = null;
	private int sfdPin;
	private IOPort fifopPort = null;
	private int fifopPin;
	private boolean currentSFD;
	private boolean currentFIFOP;

	private void setSFD(boolean sfd) {
		currentSFD = sfd;
		sfdPort.setPinState(sfdPin, sfd ? IOPort.PinState.HI : IOPort.PinState.LOW);
		if (logLevel > INFO)
			log("SFD: " + sfd + "  " + cpu.cycles);
	}

	private void setFIFOP(boolean fifop) {
		currentFIFOP = fifop;
		fifopPort.setPinState(fifopPin, fifop ? IOPort.PinState.HI : IOPort.PinState.LOW);
		if (logLevel > INFO)
			log("FIFOP: " + fifop + "  " + cpu.cycles);
	}

	public void setSFDPort(IOPort port, int pin) {
		sfdPort = port;
		sfdPin = pin;
	}

	public enum Reg {
		MODE, TXFIFO, RXFIFO
	};

	// Register addresses
	public static final int REG_MODE     = 0x00;
	public static final int REG_TXFIFO   = 0x01;
	public static final int REG_RXFIFO   = 0x02;
	
	public static final int REG_SOFF     = 0xC0;
	public static final int REG_STXON    = 0xC1;
	public static final int REG_SRXON    = 0xC2;
	public static final int REG_SFLUSHTX = 0xC3;
	public static final int REG_SFLUSHRX = 0xC4;

	// The Operation modes of the backscatter tag
	public static final int MODE_TXRX_OFF = 0x00;
	public static final int MODE_RX_ON = 0x01;
	public static final int MODE_TX_ON = 0x02;
	public static final int MODE_MAX = MODE_TX_ON;
	private static final String[] MODE_NAMES = new String[] { "off", "listen", "transmit" };

	public static final byte FLAG_WRITE = 0x40;

	// State Machine
	public enum RadioState {
		IDLE(1), RX_SFD_SEARCH(3), RX_FRAME(16), RX_DATA_TRANSFER(18), TX_FRAME(37);

		private final int state;

		RadioState(int stateNo) {
			state = stateNo;
		}

		public int getFSMState() {
			return state;
		}
	};

	public enum spiState {
		WAITING, WRITE_REGISTER, READ_REGISTER, READ_RXFIFO, WRITE_TXFIFO
	}

	private RFListener rfListener;

	private RadioState stateMachine = RadioState.IDLE;
	private spiState state = spiState.WAITING;

	private ArrayFIFO rxFIFO;
	// More than needed...
	private int[] memory = new int[512];

	private int[] txBuffer;
	private int txbufferPos = 6;
	private int bufferPos = 0;
	private int zeroSymbols = 0;
	private int rxlen = 0;
	private int rxread = 0;
	private boolean chipSelect = false;

	private CCITT_CRC txCrc = new CCITT_CRC();

	// 802.15.4 symbol period in ms
	public static final double SYMBOL_PERIOD = 0.016; // 16 us

	public BackscatterTXRadio(MSP430Core cpu) {
		super("BackscatterTXRadio", cpu);
		rxFIFO = new ArrayFIFO("RXFIFO", memory, 0, 128);
		/*
		 * page 36, CC2420 datasheet
		 * 
		 * Format of the 802.15.4 packet: Preamble: 4 bytes SFD: 1 byte Length: 1 byte
		 * Payload(PSDU): 127 bytes including FCS Packet length: 133 bytes
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

	private byte usartDataAddress;
	private byte[] registers;
	private boolean txfifoFlush;
	private byte txCursor;

	private void txNext() {
		stateMachine = RadioState.TX_FRAME;

		// The length of the packet consists of the length of the payload that has
		// been calculated and the first 6 bytes which constitute the synchronization
		// header.
		int packetLength = 6 + txBuffer[5];
		if (bufferPos < packetLength) {
			if (bufferPos == 6) {
				setSFD(true);
			}
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
			setSFD(false);
			setState(RadioState.RX_SFD_SEARCH);
			txfifoFlush = true;
		}
	}

	private boolean setState(RadioState new_state) {
		stateMachine = new_state;
//	  System.out.println("Setting state: " + new_state);

		switch (new_state) {
		case TX_FRAME:
			setMode(MODE_TX_ON);
			txNext();
			break;
		case RX_SFD_SEARCH:
			zeroSymbols = 0;
			rxFIFO.reset();
			bufferPos = 0;
			setMode(MODE_RX_ON);
			break;
		case RX_FRAME:
			rxlen = 0;
			rxread = 0;
			break;
		case IDLE:
			setMode(MODE_TXRX_OFF);
			rxFIFO.reset();
			txbufferPos = 6;
			bufferPos = 0;
			break;
		default:
			break;
		}
		return true;
	}

	public void dataReceived(USARTSource source, int data) {
		if (!chipSelect) {
			return;
		}

//	  System.out.println("Data received " + data);
		switch (state) {
		case WAITING:
			if ((data & FLAG_WRITE) != 0) {
				state = spiState.WRITE_REGISTER;
			} else {
				state = spiState.READ_REGISTER;
			}
			usartDataAddress = (byte) (data & 0x3F);
			if (usartDataAddress == REG_RXFIFO) {
				state = spiState.READ_RXFIFO;
			} else if (usartDataAddress == REG_TXFIFO) {
				state = spiState.WRITE_TXFIFO;
			}
			if ((data & 0x80) != 0) {
				state = spiState.WAITING;
				strobe(data);
			}
			break;
		case WRITE_REGISTER:
			break;
		case READ_REGISTER:
			source.byteReceived(registers[usartDataAddress]);
			state = spiState.WAITING;
			break;
		case READ_RXFIFO:
			int fifoData = rxFIFO.read();
			source.byteReceived(fifoData);
			if (currentFIFOP && rxFIFO.length() == 0) {
				setFIFOP(false);
			}
			break;
		case WRITE_TXFIFO:
			if (txfifoFlush) {
				txCursor = 0;
				txfifoFlush = false;
			}
			// TODO Check for size and overflows
			txBuffer[txCursor + 5] = data & 0xFF;
			txCursor++;
			break;
		default:
			break;
		}
	}

	private void strobe(int data) {
		switch (data) {
		case REG_SOFF:
			setState(RadioState.IDLE);
			break;
		case REG_STXON:
			if (
					stateMachine == RadioState.IDLE ||
					stateMachine == RadioState.RX_SFD_SEARCH
			) {
				setState(RadioState.TX_FRAME);
			}
			break;
		case REG_SRXON:
			if (stateMachine == RadioState.IDLE) {
				setState(RadioState.RX_SFD_SEARCH);
			}
			break;
		case REG_SFLUSHTX:
			flushTX();
			break;
		case REG_SFLUSHRX:
			flushRX();
			break;
		}
	}

	private void flushRX() {
	    rxFIFO.reset();
	    setSFD(false);
	    setFIFOP(false);
	    if( (stateMachine == RadioState.RX_SFD_SEARCH) ||
    		(stateMachine == RadioState.RX_DATA_TRANSFER) ||
	        (stateMachine == RadioState.RX_FRAME)
	        ) {
	      setState(RadioState.RX_SFD_SEARCH);
	    }
	}

	private void flushTX() {
		txbufferPos = 0;
	}

	/* Not used by the tag */
	@Override
	public int getConfiguration(int parameter) {
		return 0;
	}

	/* Not used by the tag */
	@Override
	public int getModeMax() {
		return MODE_MAX;
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

	/*
	 * Receive a byte from the radio medium
	 * 
	 * @see se.sics.mspsim.chip.RFListener#receivedByte(byte)
	 */
	public void receivedByte(byte data) {
		// Received a byte from the "air"
		// if (logLevel > INFO)
		// log("RF Byte received: " + Utils.hex8(data) + " state: " + stateMachine + "
		// noZeroes: " + zeroSymbols +
		// ((stateMachine == RadioState.RX_SFD_SEARCH || stateMachine ==
		// RadioState.RX_FRAME) ? "" : " *** Ignored"));
		//
		if (stateMachine == RadioState.RX_SFD_SEARCH) {
			// Look for the preamble (4 zero bytes) followed by the SFD byte 0x7A
			if (data == 0) {
				// Count zero bytes
				zeroSymbols++;
			} else if (zeroSymbols >= 4 && data == 0x7A) {
				// If the received byte is !zero, we have counted 4 zero bytes prior to this
				// one,
				// and the current received byte == 0x7A (SFD), we're in sync.
				// In RX mode, SFD goes high when the SFD is received
				// if (logLevel > INFO) log("RX: Preamble/SFD Synchronized.");
				setSFD(true);
				setState(RadioState.RX_FRAME);
			} else {
				/* if not four zeros and 0x7A then no zeroes... */
				zeroSymbols = 0;
				setSFD(false);
			}

		} else if (stateMachine == RadioState.RX_FRAME) {
			if (rxFIFO.isFull()) {
				rxFIFO.reset();
				setState(RadioState.RX_SFD_SEARCH);
			} else {
				rxFIFO.write(data);
				if (rxread == 0) {
					rxlen = data & 0xff;
				}
				if (rxread++ == rxlen) {
					setSFD(false);
					setFIFOP(true);
					setState(RadioState.RX_DATA_TRANSFER);
				}
			}
		}
	}

	public void setChipSelect(boolean b) {
		chipSelect = b;
		state = spiState.WAITING;
	}

	public boolean getChipSelect() {
		return chipSelect;
	}

	public void setFIFOPPort(IOPort port, int pin) {
		fifopPort = port;
		fifopPin = pin;
	}
} /* BackscatterTXRadio */
