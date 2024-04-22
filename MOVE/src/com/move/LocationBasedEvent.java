package com.move;
/*
 * ----------------------------------------------------------------------------
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
 * LocationBasedEvent.java
 * Created on May 01, 2008, 9:49 AM
 * 
 * Class containing the collection of parameters required to generate
 * a VANET event, based on a given location
 * 
 * @version 1.0
 * @author Ada Lezama<ada.lezamalugo@epfl.ch>
 */

public class LocationBasedEvent {
	// South - West coordinates of the disabled area
	private double xFrom = 0.0;
	private double yFrom = 0.0;

	// Nord - East coordinates of the disabled area
	private double xTo = 0.0;
	private double yTo = 0.0;

	// Time when the area is disabled
	private double disableRoadTime = 0;

	// Time when the area is enabled again
	private double enableRoadTime = 0;

	public double getXFrom() {
		return xFrom;
	}

	public double getYFrom() {
		return yFrom;
	}

	public double getXTo() {
		return xTo;
	}

	public double getYTo() {
		return yTo;
	}

	public double getDisableRoadTime() {
		return disableRoadTime;
	}

	public double getEnableRoadTime() {
		return enableRoadTime;
	}

	public void setXFrom(double xFrom) {
		this.xFrom = xFrom;
	}

	public void setYFrom(double yFrom) {
		this.yFrom = yFrom;
	}

	public void setXTo(double xTo) {
		this.xTo = xTo;
	}

	public void setYTo(double yTo) {
		this.yTo = yTo;
	}

	public void setDisableRoadTime(double disableRoadTime) {
		this.disableRoadTime = disableRoadTime;
	}

	public void setEnableRoadTime(double enableRoadTime) {
		this.enableRoadTime = enableRoadTime;
	}

}
