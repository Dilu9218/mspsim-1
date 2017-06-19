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

import java.util.ArrayList;

import se.sics.mspsim.core.Chip;
import se.sics.mspsim.core.EventQueue;
import se.sics.mspsim.core.TimeEvent;
//import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.core.USARTListener;
import se.sics.mspsim.core.USARTSource;
import se.sics.mspsim.util.CCITT_CRC;
import se.sics.mspsim.core.MSP430Core;

//import se.sics.mspsim.core.Chip;
//import se.sics.mspsim.util.Utils;

public class BackscatterTagRadio implements USARTListener, RFSource {
    
    public enum UartState {
        FIRST_CHAR, SECOND_CHAR
    }

    private MSP430Core cpu;
    private RFListener rfListener;
    private UartState state = UartState.FIRST_CHAR;

/* ------ Concerning transmission -------------------  */    
    private int[] txBuffer;
    private int dataValue = 0;
    private int payload = 0;
    private int payloadLength = 0;
    private int intermediateValue = 0;
    private boolean startSending = false;
    private boolean nextLevel = false;
    
    private int txbufferPos = 6;
    private int bufferPos = 0;
    private CCITT_CRC txCrc = new CCITT_CRC();
/* --------------------------------------------------- */
    
/* ------ Concerning time and time events -----------  */    

 // 802.15.4 symbol period in ms    
public static final double SYMBOL_PERIOD = 0.016;

    // Check MSP430Config.java
    private final int MAX_DCO_FRQ = 4915200;
    
    private EventQueue vTimeEventQueue = new EventQueue();
    
    private EventQueue cycleEventQueue = new EventQueue();
    private long nextCycleEventCycles;
    
    private long nextEventCycles;
    private long nextVTimeEventCycles;
    private long cycles = 0;
    private long lastCyclesTime = 0;
    private long lastVTime = 0;
    private double currentDCOFactor = 1.0;
    private ArrayList<BackscatterTagRadio>  tags = new ArrayList<BackscatterTagRadio>();
/* ---------------------------------------------------- */


    public BackscatterTagRadio() {
/**/    System.out.println("BackscatterTagRadio");
            
        //super.addTag(this);
        addTag(this);
        
        /*
         * Format of the 802.15.4 packet:
         * Preamble: 4 bytes
         * SFD: 1 byte
         * Length: 1 byte
         * Payload(PSDU): 127 bytes including FCS
         */
        txBuffer = new int[133];
        // Set the preamble and sfd for protocol 802.15.4
        txBuffer[0] = 0;
        txBuffer[1] = 0;
        txBuffer[2] = 0;
        txBuffer[3] = 0;
        txBuffer[4] = 0x7A;
        // Contains the length of the PPDU 
        txBuffer[5] = 0;
    }
    
    public void addTag(BackscatterTagRadio tag) {
        tags.add(tag);
    }
  
    
    
//    
//    public TimeEvent sendEvent = new  TimeEvent(0, "BackscatterTag Send") {
//        public void execute(long t) {
///**/        System.out.println("sendEvents.txNext");
//            txNext();
//        }
//    };
    
//    private long getTime() {
//        long diff = cycles - lastCyclesTime;
//        return lastVTime + (long) (diff * currentDCOFactor);
//    }
//    
//    // Converts a virtual time to a cycles time according to the current cycle speed
//    private long convertVTime(long vTime) {
//      long tmpTime = lastCyclesTime + (long) ((vTime - lastVTime) / currentDCOFactor);
//      //System.out.println("ConvertVTime: vTime=" + vTime + " => " + tmpTime);
//      return tmpTime;
//    }

//    private void executeEvents() {
//        /**/ System.out.println("executeEvents");      
//            if (cycles >= nextVTimeEventCycles) {
//              if (vTimeEventQueue.eventCount == 0) {
//                nextVTimeEventCycles = cycles + 10000;
//              } else {
//                TimeEvent te = vTimeEventQueue.popFirst();
//                long now = getTime();
////                if (now > te.time) {
////                  System.out.println("VTimeEvent got delayed by: " + (now - te.time) + " at " +
////                      cycles + " target Time: " + te.time + " class: " + te.getClass().getName());
////                }
//                te.execute(now);
//                if (vTimeEventQueue.eventCount > 0) {
//                  nextVTimeEventCycles = convertVTime(vTimeEventQueue.nextTime);
//                } else {
//                  nextVTimeEventCycles = cycles + 10000;          
//                }
//              }
//            }
//            
//            if (cycles >= nextCycleEventCycles) {
//              if (cycleEventQueue.eventCount == 0) {
//                nextCycleEventCycles = cycles + 10000;
//              } else {
//                TimeEvent te = cycleEventQueue.popFirst();
//                te.execute(cycles);
//                if (cycleEventQueue.eventCount > 0) {
//                  nextCycleEventCycles = cycleEventQueue.nextTime;
//                } else {
//                  nextCycleEventCycles = cycles + 10000;          
//                }
//              }
//            }
//            
//            // Pick the one with shortest time in the future.
//            nextEventCycles = nextCycleEventCycles < nextVTimeEventCycles ? 
//                nextCycleEventCycles : nextVTimeEventCycles;
//          }
    
    /**
     * Schedules a new Time event using the virtual time clock
     * @param event
     * @param time
     */
//    private void scheduleTimeEvent(TimeEvent event, long time) {
///**/  System.out.println("scheduleTimeEvent");        
//      long currentNext = vTimeEventQueue.nextTime;
//      vTimeEventQueue.addEvent(event, time);
//      if (currentNext != vTimeEventQueue.nextTime) {
//        // This is only valid when not having a cycle event queue also...
//        // if we have it needs to be checked also!
//        nextVTimeEventCycles = convertVTime(vTimeEventQueue.nextTime);
//        if (nextEventCycles > nextVTimeEventCycles) {
//          nextEventCycles = nextVTimeEventCycles;
//        }
//        /* Warn if someone schedules a time backwards in time... */
//        if (cycles > nextVTimeEventCycles) {
//          //logger.logw(this, WarningType.EMULATION_ERROR, "Scheduling time event backwards in time!!!");
//          throw new IllegalStateException("Cycles are passed desired future time...");
//        }
//      }
//    }
//    
//    
//    /**
//     * Schedules a new Time event msec milliseconds in the future
//     * @param event
//     * @param time
//     */
//    private long scheduleTimeEventMillis(TimeEvent event, double msec) {
///**/  System.out.println("scheduleTimeEventMillis");        
//      /*    System.out.println("MAX_DCO " + bcs.getMaxDCOFrequency());*/
//      long time = (long) (getTime() + msec / 1000 * MAX_DCO_FRQ);
//      //System.out.println("Scheduling at: " + time + " (" + msec + ") getTime: " + getTime());
//      scheduleTimeEvent(event, time);
//      return time;
//    }

    
    
    private void txNext() {
        // The length of the packet consists of the length of the payload the has been measured
        // plus the standard first 6 bytes which constitute the synch header. 
        int packetLength = 6 + payloadLength;
 /**/   System.out.println("packetLength: " + packetLength);  
 /**/   //System.out.println("txBuffer: " + txBuffer.length );
        if (bufferPos < packetLength) {
/**/        System.out.println("Tag.txNext");
            if(bufferPos == packetLength - 2) {
                txCrc.setCRC(0);
                for (int i = 6; i < packetLength-2; i++) {
                    txCrc.addBitrev(txBuffer[i]);
                }
                txBuffer[packetLength - 2] = txCrc.getCRCHi();
/**/            System.out.printf("txbuffer[%d] = %d\n", packetLength - 2, txBuffer[packetLength - 2]);
                txBuffer[packetLength - 1] = txCrc.getCRCLow();
/**/            System.out.printf("txbuffer[%d] = %d\n", packetLength - 1, txBuffer[packetLength - 1]);                
            }
            
            if (rfListener != null) {
/**/            System.out.println("tag.rfListener");
                rfListener.receivedByte((byte)(txBuffer[bufferPos] & 0xFF));
/**/            System.out.println(txBuffer[bufferPos] + " sent");
            }
            
/**/        System.out.printf("txbuffer[%d] = %d\n", bufferPos, (byte)txBuffer[bufferPos]);
            bufferPos++;
/**///        System.out.println("get here!");
            txNext();
            //cpu.scheduleTimeEventMillis(sendEvent, SYMBOL_PERIOD * 2);
            //scheduleTimeEventMillis(sendEvent, SYMBOL_PERIOD * 2);
        } else {
            bufferPos = 0;
        }
    }
    
    
    public void dataReceived(USARTSource source, int data) {
/**/System.out.println("BackscatterTag.dataReceived");
               
        // you need to send s\n before you start any sending
        if ((data & 0xFF) == 's') {
/**/        System.out.println("s came");            
            nextLevel = true;
        }
        else if ( (data & 0xFF) == '\n' & nextLevel) {
/**/        System.out.println("new line came");
            nextLevel = false;
            startSending = true;
            return;
        }

        // 126 denotes 125 times for payload plus 1 for the detection of the new line character.
        if (startSending & payload < 126) {
            if ((data & 0xFF) != '\n') {
/**/            System.out.println("data1: " + (data & 0xFF));
                if ((data & 0xFF) <= 57) {
/**/                System.out.println("number(0-9)");
                    // Subtracting 48 from an ascii number(0-9) gives you the
                    // binary representation that this number represents
                    intermediateValue = (data & 0xFF) - 48 ;
                } else if ((data & 0xFF) <= 70) {
/**/                System.out.println("letter(A-F)");
                    // Subtracting 55 from an ascii letter(A-F) gives you the
                    // binary representation of the number that this upper case 
                    //letter represents.
                    intermediateValue = (data & 0xFF) - 55;
                } else if ((data & 0xFF) <= 102) {
/**/                System.out.println("letter(a-f)");                    
                 // Subtracting 55 from an ascii letter(a-f) gives you the
                    // binary representation of the number that this lower case 
                    //letter represents.
                    intermediateValue = (data & 0xFF) - 87;
                }
                
                switch(state) {
                case FIRST_CHAR:
/**/                System.out.println("FIRST_CHAR");
                    dataValue = intermediateValue << 4;
                    state = UartState.SECOND_CHAR;
                    break;
                    
                case SECOND_CHAR:
/**/                System.out.println("SECOND_CHAR");                    
                    dataValue |= intermediateValue & 0x0F;
/**/                System.out.println("txbufferPos = " + txbufferPos);
                    txBuffer[txbufferPos++] = (dataValue & 0xFF);
/**/                System.out.printf("txBuffer[%d] = %d\n ", txbufferPos-1, txBuffer[txbufferPos-1]);
                    payload++;
/**/                System.out.println("payload = " + payload);
                    //txbufferPos++;
                    state = UartState.FIRST_CHAR;
                    break;
                }
                
            } else {
/**/            System.out.println("data2: " + data);
                // txBuffer[5] contains the length of the PPSDU(p.36). Within this length 
                // the 2 last ints concerning the CRC, are also included.
                // The most significant bit is reserved, based on 802.15.4 specification,
                // and should be set to zero.
                payloadLength = payload + 2;
                txBuffer[5] = (payloadLength & 0x7F) ;
/**/            System.out.println("txBuffer[5] = " + txBuffer[5]);
                // transmit
                txNext();

                //TO CHECK: if I have to empty the buffer by myself each time the packet is
                //transmitted or all packets will always have the same length?
                payload = 0 ;
                txbufferPos = 6;
                startSending = false;
                state = UartState.FIRST_CHAR;
/**/            System.out.println("Transmission is done!");    
            }
        } else {
            //logw(WarningType.EXECUTION, "**** Warning - packet size too large - repeating packet ints txfifoPos: " + txfifoPos);
/**/        System.out.println(!startSending ? "No packet received by the tag" : "Warning - packet size too large");
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
    
    
} /* BackscatterTagRadio */
