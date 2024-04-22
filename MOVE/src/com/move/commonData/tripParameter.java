package com.move.commonData;
/**
 * Thesis implementation
 * Class : tripParameter.java
 * Writen By : ZhiHai(Harris) Mo
 * Student Number : 3057489
 **/

public class tripParameter {

	String id = "null";
	String depart = "null";
	String from = "null";
	String to = "null";
	String type = "null";
	String period = "null";
	String repno = "null";

	//<tripdef  id="<ID>" depart="0" from="1i" to="2o" type="0" period="100" repno="10"/>

	public tripParameter(String identification, String tripDepart, String tripFrom, String tripTo, String tripPeriod, String tripType,
			String triRepno) {

		id = new String(identification);
		depart = new String(tripDepart);
		from = new String(tripFrom);
		to = new String(tripTo);
		type = new String(tripType);
		period = new String(tripPeriod);
		repno = new String(triRepno);

	}

	public String getType() {

		return type;
	}

	public String getPeriod() {

		return period;
	}

	public String getRepno() {

		return repno;
	}

	public String getID() {
		return id;
	}

	public String getDepart() {
		return depart;
	}

	public String getFrom() {
		return from;
	}

	public String getTo() {
		return to;
	}

}
