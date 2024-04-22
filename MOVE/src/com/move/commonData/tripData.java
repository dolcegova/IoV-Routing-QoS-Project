package com.move.commonData;
/**
 * Thesis implementation
 * Class : tripData.java
 * Writen By : ZhiHai(Harris) Mo
 * Student Number : 3057489
 **/

import java.util.*;

public class tripData {

	ArrayList allTrips;
	boolean empty;

	public tripData() {
		allTrips = new ArrayList();
	}

	public ArrayList returnData() {
		return allTrips;
	}

	public void addTrips(tripParameter trip) {
		allTrips.add(trip);
	}

	public void clearNodes() {
		allTrips.clear();
	}

	public boolean isEmpty() {
		empty = allTrips.isEmpty();
		return empty;
	}

	public void deleteTrip(int index) {
		allTrips.remove(index);
	}

}
