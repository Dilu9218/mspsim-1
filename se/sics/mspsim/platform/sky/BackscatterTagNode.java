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
 *
 * Author  : George Daglaridis
 * Author  : Carlos Perez Penichet
 *
 */

package se.sics.mspsim.platform.sky;

import se.sics.mspsim.chip.BackscatterTXRadio;
import se.sics.mspsim.core.IOPort;
import se.sics.mspsim.core.USARTSource;

public class BackscatterTagNode extends SkyNode {
	
  // Battery-free tag chip select pin: P2.0
  public static int TAG_CHIP_SELECT = 1;
  // Battery-free tag FIFOP pin: P1.0
  public static int TAG_FIFOP = 0;
  // Battery-free tag SFD pin: P4.1
  public static int TAG_SFD = 1;
    
  public BackscatterTXRadio tag;
    
  public BackscatterTagNode() {
      super();
  }
    
  //USARTListener
  @Override
  public void dataReceived(USARTSource source, int data) {
	  super.dataReceived(source, data);
	  tag.dataReceived(source, data);
  }
  
  @Override
	public void portWrite(IOPort source, int data) {
		super.portWrite(source, data);
		if (source == port2) {
            // Chip select = active low...
            tag.setChipSelect((data & TAG_CHIP_SELECT) == 0);
		}
	}
    
  @Override
  public void setupNodePorts() {
    super.setupNodePorts();
      
    /* Creation of the Backscatter TX Radio module */
    tag = new BackscatterTXRadio(cpu);
    
    port1.addPortListener(this);
    tag.setFIFOPPort(port1, TAG_FIFOP);
    tag.setSFDPort(port4, TAG_SFD);
  }
}