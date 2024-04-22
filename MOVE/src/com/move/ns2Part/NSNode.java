package com.move.ns2Part;
public class NSNode {

	String id;
	String time;
	double x;
	double y;
	String temp = "";
	String timestep = " ";
	String speed = " ";
	String connectionState = "sink";

	public NSNode(String timeIndex, String identification, double xCoord,
			double yCoord) {
		timestep = new String(timeIndex);
		id = new String(identification);
		x = xCoord;
		y = yCoord;

	}

	public String getID() {
		return id;
	}

	public String getNumberID() {
		temp = "";
		for (int ind = 0; ind < id.length(); ind++) {
			int a = id.charAt(ind);
			temp = temp + a;
		}
		return temp;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setSpeed(String s) {
		speed = s;
	}

	public String getSpeed() {
		return speed;
	}

	public void setTimestep(String s) {
		timestep = s;
	}

	public String getTimestep() {
		return timestep;
	}

	public void setState(String s) {
		connectionState = s;
	}

	public String getState() {
		return connectionState;
	}

}
