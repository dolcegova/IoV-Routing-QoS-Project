package com.move.commonData;
/* ----------------------------------------------------------------------------
 * Copyright 2006-2008,
 * Ada Lezama Lugo, Michal Piorkowski, Maxim Raya, EPFL-I&C-LCA
 * All rights reserved
 * ----------------------------------------------------------------------------
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------------
 */

/**
 * VehicleBasedEvent.java
 * Created on May 01, 2008, 9:49 AM
 * 
 * Class containing the collection of parameters required to generate
 * a VANET event, based on a given vehicle
 * 
 * @version   1.0
 * @author    Ada Lezama<ada.lezamalugo@epfl.ch>
 */

public class VehicleBasedEvent {
    
    private String nodeID          = "un-assign";
    private int startBrcTime    = -1;
    private int stopBrcTime     = -1;
    
    public VehicleBasedEvent(String nodeID, int startBrcTime, int stopBrcTime){
        this.nodeID = nodeID;
        this.startBrcTime = startBrcTime;
        this.stopBrcTime = stopBrcTime;
    }

    public String getNodeID() {
        return nodeID;
    }

    public int getStartBrcTime() {
        return startBrcTime;
    }

    public int getStopBrcTime() {
        return stopBrcTime;
    }

    public void setNodeID(String nodeID) {
        this.nodeID = nodeID;
    }

    public void setStartBrcTime(int startBrcTime) {
        this.startBrcTime = startBrcTime;
    }

    public void setStopBrcTime(int stopBrcTime) {
        this.stopBrcTime = stopBrcTime;
    }
    

    
}
