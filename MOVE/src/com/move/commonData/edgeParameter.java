package com.move.commonData;
/**
 * Thesis implementation
 * Class : edgeParameter.java
 * Writen By : ZhiHai(Harris) Mo
 * Student Number : 3057489
 **/

public class edgeParameter {

	String edgeID;

	String fromnode;
	String tonode;

	String priority = "";
	int priorityFlag;

	String nolanes = "";
	int nolanesFlag;

	String speed = "";
	int speedFlag;

	String length = "";
	int lengthFlag;

	String typeID = "";
	public int typeIDflag;

	String shape = "";
	int shapeFlag;

	String spread_type = "";
	int spread_typeFlag;

	int typeFlag;

	String xfrom;
	String yfrom;
	String xto;
	String yto;

	public edgeParameter(String identification, String startNode,
			String endNode) {
		typeFlag = 1;
		edgeID = new String(identification);
		fromnode = new String(startNode);
		tonode = new String(endNode);
		priorityFlag = 0;
		nolanesFlag = 0;
		speedFlag = 0;
		lengthFlag = 0;
		typeIDflag = 0;
		shapeFlag = 0;
		spread_typeFlag = 0;
	}

//      old version before SUMO 0.11.0
//      public edgeParameter(String identification,String xPositionf,
//                          String yPositionf,String xPositiont,String yPositiont){    	  
//              typeFlag = 2;
//              edgeID   = new String(identification);
//              xfrom    = xPositionf;
//              yfrom    = yPositionf;
//              xto      = xPositiont;
//              yto      = yPositiont;
//
//              priorityFlag = 0;
//              nolanesFlag  = 0;
//              speedFlag    = 0;
//              lengthFlag   = 0;
//              typeIDflag   = 0;
//              shapeFlag    = 0;
//              spread_typeFlag = 0;
//
//       }

	public void setNolanes(String numberLanes) {
		nolanesFlag = 1;
		nolanes = new String(numberLanes);
	}

	public void setSpeed(String vehicleSpeed) {
		speedFlag = 1;
		speed = new String(vehicleSpeed);
	}

	public void setPriority(String edgePriority) {
		priorityFlag = 1;
		priority = new String(edgePriority);
	}

	public void setLength(String edgeLength) {
		lengthFlag = 1;
		length = new String(edgeLength);
	}

	public void setShape(String edgeShape) {
		shapeFlag = 1;
		shape = new String(edgeShape);
	}

	public void setSpread(String edgeSpead) {
		spread_typeFlag = 1;
		spread_type = new String(edgeSpead);
	}

	public void setType(String edgeTypeID) {
		typeIDflag = 1;
		typeID = new String(edgeTypeID);
	}

	public int getTypeFlag() {
		return typeFlag;
	}

	public String getEdgeID() {
		return edgeID;
	}

	public String getFromnode() {
		return fromnode;
	}

	public String getTonode() {
		return tonode;
	}

	public String getXfrom() {
		return xfrom;
	}

	public String getYfrom() {
		return yfrom;
	}

	public String getXto() {
		return xto;
	}

	public String getYto() {
		return yto;
	}

	public String getNolanes() {
		return nolanes;
	}

	public int getNolanesFlag() {
		return nolanesFlag;
	}

	public String getSpeed() {
		return speed;
	}

	public int getSpeedFlag() {
		return speedFlag;
	}

	public String getPriority() {
		return priority;
	}

	public int getPriorityFlag() {
		return priorityFlag;
	}

	public int getLengthFlag() {
		return lengthFlag;
	}

	public String getLength() {
		return length;
	}

	public int getShapeFlag() {
		return shapeFlag;
	}

	public String getShape() {
		return shape;
	}

	public int getSpreadFlag() {
		return spread_typeFlag;
	}

	public String getSpread() {
		return spread_type;
	}

	public int getTypeIDFlag() {
		return typeIDflag;
	}

	public String getTypeID() {
		return typeID;
	}

}
