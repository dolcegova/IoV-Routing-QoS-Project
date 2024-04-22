package com.move;
import java.io.*;
import java.util.*;

import com.move.commonData.TurnData;
import com.move.commonData.TurnEdge;
import com.move.commonData.TurnInterval;
import com.move.commonData.TurnParameter;
import com.move.commonData.edgeData;
import com.move.commonData.edgeParameter;
import com.move.commonData.flowData;
import com.move.commonData.flowParameter;
import com.move.commonData.nodeData;
import com.move.commonData.nodeParameter;
import com.move.commonData.routeData;
import com.move.commonData.routeParameter;
import com.move.commonData.tripData;
import com.move.commonData.tripParameter;
import com.move.commonData.typeData;
import com.move.commonData.typeParameter;
import com.move.commonData.vehicleParameter;
import com.move.commonData.vtypeParameter;

/*
	Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
	UNSW - Sydney
*/

public class construct {
	     String xpoint;
	     String ypoint;
	     String type;
	     String nodeID;

	     String con;
	     String SName;
	     
	     PrintWriter  printer;

	     ArrayList Alld;
	     ArrayList routeList;
	     ArrayList vtypeList;
	     ArrayList vehicleList;
	     
	     int isdefault = 0;	     

	     public construct(nodeData d,String Name){
		    
		    Alld = d.returnData();
		    SName = new String(Name);
		    constructNode();
	     }
	     
	     public construct(typeData d,String Name){
		    
		    Alld = d.returnData();
		    SName = new String(Name);
		    constructType();
	     }

	     public construct(edgeData d,String Name){
		    
		    Alld = d.returnData();
		    SName = new String(Name);
		    constructEdge();
	     }

	     public construct(tripData d,String Name){
		    Alld = d.returnData();
		    SName = new String(Name);
		    constructTrip();	       
	     }
	     
	     public construct(routeData d,String Name){

		    routeList = d.returnRoute();
		    vtypeList = d.returnType();
		    vehicleList = d.returnVehicles();
		    SName = new String(Name);
		    constructRoute();
	     }	     
	     
	     public void constructRoute(){

		int l;

		try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 vtypeParameter t;
		 routeParameter r;
		 vehicleParameter v;

		 l = vtypeList.size();

		 printer.print("<routes>\n");
		 for(int i=0; i < l; i++){
		     t = (vtypeParameter)vtypeList.get(i);
		     printer.print("   <vtype id=\""+t.getID()+"\"");
		     printer.print(" accel=\""+t.getAccel()+"\"");
		     printer.print(" decel=\""+t.getDecel()+"\"");
		     printer.print(" sigma=\""+t.getSigma()+"\"");
		     printer.print(" length=\""+t.getLength()+"\"");
		     printer.print(" maxspeed=\""+t.getMaxspeed()+"\"/>\n");
		 }

		 l = routeList.size();
		 for(int i=0; i < l; i++){
		     r = (routeParameter)routeList.get(i);
		     printer.print("   <route id=\""+r.getID()+"\"");
		     printer.print(" multi_ref=\""+r.getMulti_ref()+"\">");
		     printer.print(r.getPath()+"</route>\n");

		 }

		 l = vehicleList.size();
		 for(int i=0; i < l; i++){
		     v = (vehicleParameter)vehicleList.get(i);
		     printer.print("   <vehicle id=\""+v.getID()+"\"");
		     printer.print(" type=\""+v.getType()+ "\"");
		     printer.print(" route=\""+v.getRoute()+"\"");
		     printer.print(" depart=\""+v.getDepart()+"\"");
		     printer.print(" period=\""+v.getPeriod()+"\"");
		     printer.print(" repno=\""+v.getRepno()+"\"/>\n");
		 }
		 printer.print("</routes>\n");

		 printer.close();
	     }
	     
 
	 
	     public construct(String Filename, String Nodes, String Edges, String Types, String Output, int isd,String type, String nolanes,String speed,String priority){     	     
		 try{
		     printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(Filename)));
		 }
		 catch(IOException e){}
		 
//			======================================== old version part (sumo 0.10.3)
//		 printer.print("<configuration>\n\n");
//		 printer.print("    <files>\n");
//		 printer.print("       <xml-node-files>"+Nodes+"</xml-node-files>\n");
//		 printer.print("       <xml-edge-files>"+Edges+"</xml-edge-files>\n");
//		 printer.print("       <xml-connection-files></xml-connection-files>\n");
//		 printer.print("       <type-file>"+Types+"</type-file>\n\n");
//		 printer.print("       <output-file>"+Output+"</output-file>\n");
//		 printer.print("    </files>\n\n");
//
//		 if (isd == 0){
//		  printer.print("    <defaults>\n");
//		  printer.print("	 <type>Unknown</type>\n");
//		  printer.print("	 <lanenumber>1</lanenumber>\n");
//		  printer.print("	 <speed>13.9</speed>\n");
//		  printer.print("	 <priority>1</priority>\n");
//		  printer.print("	 <capacity-norm></capacity-norm>\n");
//		  printer.print("    </defaults>\n");
//		 }
//		 else if (isd == 1){
//		  printer.print("    <defaults>\n");
//		  printer.print("	 <type>"+type+"</type>\n");
//		  printer.print("	 <lanenumber>"+nolanes+"</lanenumber>\n");
//		  printer.print("	 <speed>"+speed+"</speed>\n");
//		  printer.print("	 <priority>"+priority+"</priority>\n");
//		  printer.print("	 <capacity-norm></capacity-norm>\n");
//		  printer.print("    </defaults>\n");
//		 }
//		 
//		 printer.print("    <reports>\n");
//		 printer.print("	<verbose></verbose>\n");
//		 printer.print("	<print-options></print-options>\n");
//		 printer.print("    </reports>\n");
//
//		 printer.print("    <process>\n");
//		 printer.print("       <remove-geometry>x</remove-geometry>\n");
//		 printer.print("    </process>\n");
//
//		 printer.print("</configuration>");
//		 printer.close();
//			======================================== old version part (sumo 0.10.3)
		 
		 printer.print("<configuration>\n\n");
		 printer.print("    <input>\n");
		 printer.print("       <xml-node-files value=\"" + Nodes + "\"/>\n");
		 printer.print("       <xml-edge-files value=\"" + Edges + "\"/>\n");
		 printer.print("       <xml-connection-files value=\"\"/>\n");
		 printer.print("       <xml-type-files value=\""+Types+"\"/>\n");
		 printer.print("    </input>\n\n");
		 
		 printer.print("    <output>\n");		 
		 printer.print("       <output-file value=\"" + Output + "\"/>\n");
		 printer.print("    </output>\n\n");

		 if (isd == 0){
		  printer.print("    <defaults>\n");
		  printer.print("       <type value=\"Unknown\"/>\n");
		  printer.print("       <lanenumber value=\"1\"/>\n");
		  printer.print("       <speed value=\"13.9\"/>\n");
		  printer.print("       <priority value=\"1\"/>\n");
		  printer.print("       <capacity-norm value=\"\"/>\n");
		  printer.print("    </defaults>\n\n");
		 }
		 else if (isd == 1){
		  printer.print("    <defaults>\n");
		  printer.print("       <type value=\"" + type + "\"/>\n");
		  printer.print("       <lanenumber value=\"" + nolanes + "\"/>\n");
		  printer.print("       <speed value=\"" + speed + "\"/>\n");
		  printer.print("       <priority value=\"" + priority + "\"/>\n");
		  printer.print("       <capacity-norm value=\"\"/>\n");
		  printer.print("    </defaults>\n\n");
		 }
		 
		 
		 printer.print("    <reports>\n");		 
		 printer.print("       <print-options value=\"false\"/>\n");
		 printer.print("    </reports>\n\n");
	     
		 printer.print("    <processing>\n");
		 printer.print("       <speed-in-kmh value=\"false\"/>\n");
		 printer.print("       <no-turnarounds value=\"false\"/>\n");		 
		 printer.print("       <remove-geometry value=\"false\"/>\n");
		 printer.print("       <remove-isolated value=\"false\"/>\n");
		 printer.print("    </processing>\n\n");
		 printer.print("</configuration>");
		 printer.close();
	     }


	     public construct(String NetFile, String RouteFile, String NetState, String Output, String begin, String end){	     	     
		 try{
		     printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(Output)));
		 }
		 catch(IOException e){}

//			======================================== old version part (sumo 0.10.3)	 
//		 printer.print("<configuration>\n\n");
//		 printer.print("    <files>\n");
//		 printer.print("      <net-file>"+NetFile+"</net-file>\n");
//		 printer.print("      <route-files>"+RouteFile+"</route-files>\n");
//		 printer.print("      <additional-files></additional-files>\n");
//		 printer.print("      <junction-files></junction-files>\n");
//		 printer.print("      <netstate-dump>"+NetState+"</netstate-dump>\n");
//		 printer.print("      <output-file></output-file>\n");
//		 printer.print("      <dump-intervals></dump-intervals>\n");
//		 printer.print("      <dump-basename></dump-basename>\n");
//		 printer.print("    </files>\n\n");
//
//		 printer.print("    <simulation>\n");
//		 printer.print("	<begin>"+begin+"</begin>\n");
//		 printer.print("	<end>"+end+"</end>\n");
//		 printer.print("	<route-steps></route-steps>\n");
//		 printer.print("    </simulation>\n\n");
//		 printer.print("    <reports>\n");
//		 printer.print("       <verbose></verbose>\n");
//		 printer.print("       <print-options></print-options>\n");
//		 printer.print("    </reports>\n");
//		 printer.print("</configuration>\n");
//		 printer.close();
//			======================================== old version part (sumo 0.10.3)		 
		 
		 printer.print("<configuration>\n\n");
		 printer.print("    <input>\n");
		 printer.print("      <net-file value=\"" + NetFile + "\"/>\n");
		 printer.print("      <route-files value=\"" + RouteFile + "\"/>\n");
		 printer.print("      <additional-files value=\"\"/>\n");
		 printer.print("      <junction-files value=\"\"/>\n");		 
		 printer.print("    </input>\n\n");

		 printer.print("    <output>\n");
		 printer.print("      <netstate-dump value=\"" + NetState + "\"/>\n");
		 printer.print("      <tripinfo-output value=\"output-tripinfos.xml\"/>\n");
		 printer.print("      <emissions-output value=\"output-emissions.xml\"/>\n");
		 printer.print("      <vehroute-output value=\"output-vehroutes.xml\"/>\n");
		 printer.print("    </output>\n\n");
		 		 
		 printer.print("    <time>\n");
		 printer.print("      <begin value=\"" + begin + "\"/>\n");
		 printer.print("      <end value=\"" + end + "\"/>\n");
		 printer.print("      <time-to-teleport value=\"-1\"/>\n");
		 printer.print("      <srand value=\"23423\"/>\n");
		 printer.print("      <route-steps value=\"-1\"/>\n");		  
		 printer.print("    </time>\n\n");
		 
		 printer.print("    <reports>\n");		 
		 printer.print("       <print-options value=\"false\"/>\n");
		 printer.print("    </reports>\n\n");	 
		 
		 printer.print("</configuration>\n");
		 printer.close();
	     }
	     
	     public construct(TurnData d,String Name){
		    Alld = d.returnData();
		    SName = new String(Name);		    
		    constructTurn();
	     }
	     
	     public construct(flowData d,String Name){
		    Alld = d.returnData();
		    SName = new String(Name);
		    constructFlow();	       
	     }
	     
	     public void constructType(){
		 int l;
		 int i;

		 try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 typeParameter tempType;
		 l = Alld.size();
		 
		 printer.print("<types>\n");

		 for(i=0; i < l; i++){

		     tempType = (typeParameter) Alld.get(i);
		     String temp = new String(tempType.getID());
		     printer.print("   <type id=\""+temp+"\"");

		     temp = new String(tempType.getName());
		     printer.print(" name=\""+temp+"\"");

		     temp = new String(tempType.getPriority());
		     printer.print(" priority=\""+temp+"\"");

		     
		     temp = new String(tempType.getNolanes());
		     printer.print(" nolanes=\""+temp+"\"");

		     temp = new String(tempType.getSpeed());
		     printer.print(" speed=\""+temp+"\"");
		     
		     temp = new String(tempType.getCapacity());
		     printer.print(" Capacity=\""+temp+"\"");
		     
		     temp = new String(tempType.getParkingPossible());
		     printer.print(" ParkingPossible=\""+temp+"\" />\n");
		 }

		 printer.print("</types>\n");

		 printer.close();
	     }
	     
	     public void constructTurn(){
		 int l;
		 int i;
		 int j;
		 int k;
		 int m;
		 int n;
		 
		 String temp1;
		 String temp2;
		 
		 String temp3;
		 
		 String temp4;
		 String temp5;
		 
		 try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}
		 
		 TurnParameter tempPerc;
		 TurnEdge      tempEdge;
		 TurnInterval  tempInterval;
		 
		 ArrayList alle;
		 ArrayList allp;
		 
		 l= Alld.size();
		 
		 printer.print("<turn-defs>\n");
		 for(i=0; i<l; i++){
		    tempInterval = (TurnInterval) Alld.get(i);
		    temp1 = tempInterval.getBegin();
		    temp2 = tempInterval.getEnd();
		    printer.print("<interval begin=\""+temp1+"\" end=\""+temp2+"\">\n");
		    
		    alle =  tempInterval.returnData();
		    k = alle.size();
		    
		    for(j=0; j<k; j++){
		    
		       tempEdge = (TurnEdge) alle.get(j);
		       temp3 = tempEdge.getID();   
		       printer.print("	    <fromedge id=\""+temp3+"\">\n");
		       
		       allp = tempEdge.returnData();
		       m = allp.size();
		       for(n=0; n<m; n++){
			   tempPerc = (TurnParameter) allp.get(n);
			   temp4 = tempPerc.getID();
			   temp5 = tempPerc.getPerc();
			   printer.print("		<toedge id=\""+temp4+"\" probability=\""+temp5+"\"></toedge>\n");
			   //printer.print("		<toedge id=\""+temp4+"\" perc=\""+temp5+"\"></toedge>\n");
		       }
		       printer.print("	    </fromedge>\n");   
		    }
		    printer.print("</interval>\n");
		  }
		  printer.print("</turn-defs>\n");
		  printer.close();
	     }
		    
		    
		 
	     
	     public void constructFlow(){
		 int l;
		 int i;

		 try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 flowParameter tempFlow;
		 l = Alld.size();


		 con = new String("<flows>\n");
		 printer.print(con);

		 for(i=0; i < l; i++){

		     tempFlow = (flowParameter) Alld.get(i);
		     String temp = new String(tempFlow.getID());


		     printer.print("<flow id=\""+ temp+"\"" );
		     

		     temp = new String(tempFlow.getFrom());
		     printer.print(" from=\""+temp+"\"");

		     temp = new String(tempFlow.getTo());
		     printer.print(" to=\""+temp+"\"");

		     temp = new String(tempFlow.getBegin());
		     printer.print(" begin=\""+temp+"\"");
		     
		     temp = new String(tempFlow.getEnd());
		     printer.print(" end=\""+temp+"\"");
		     
		     temp = new String(tempFlow.getNo());
		     printer.print(" no=\""+temp+"\" />\n");
		 }

		 printer.print("</flows>\n");

		 printer.close();
	     }

	     public void constructNode(){
		 int l;
		 int i;

		 try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 nodeParameter tempNode;
		 l = Alld.size();


		 con = new String("<nodes>\n");
		 printer.print(con);

		 for(i=0; i < l; i++){
		     tempNode = (nodeParameter) Alld.get(i);
		     String temp = new String(tempNode.getID());

		     printer.print("<node id=\"");
		     printer.print(temp);
		     printer.print("\" x=\"");

		     String temp1 = new String(tempNode.getX());		     
		     printer.print(temp1+"\" y=\"");

		     String temp2 = new String(tempNode.getY());		     
		     printer.print(temp2+"\" type=\"");

		     String temp3 = new String(tempNode.getType());		     
		     printer.print(temp3+"\"/>\n");
		 }

		 printer.print("</nodes>\n");

		 printer.close();
	     }

	     public void constructTrip(){
		 int l;
		 int i;

		 try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 tripParameter tempTrip;
		 l = Alld.size();


		 con = new String("<tripdefs>\n");
		 printer.print(con);

		 for(i=0; i < l; i++){

		     tempTrip = (tripParameter) Alld.get(i);
		     String temp = new String(tempTrip.getID());


		     printer.print("<tripdef id=\""+ temp+"\"" );
		     
		     temp = new String(tempTrip.getDepart());
		     printer.print(" depart=\""+temp+"\"");

		     temp = new String(tempTrip.getFrom());
		     printer.print(" from=\""+temp+"\"");

		     temp = new String(tempTrip.getTo());
		     printer.print(" to=\""+temp+"\"");
		     
		     temp = new String(tempTrip.getType());
		     printer.print(" type=\""+temp+"\"");
		     
		     temp = new String(tempTrip.getPeriod());
		     printer.print(" period=\""+temp+"\"");
		     
		     temp = new String(tempTrip.getRepno());
		     printer.print(" repno=\""+temp+"\" />\n");
		 }

		 printer.print("</tripdefs>\n");

		 printer.close();
	     }
	     
	     
	     
	     
	     
	    public void constructEdge(){
		int l;
		int i;

		try{
		 printer = new PrintWriter
		     (new BufferedWriter(new FileWriter(SName)));
		 }
		 catch(IOException e){}

		 edgeParameter tempEdge;
		 l = Alld.size();

		 con = new String("<edges>\n");
		 printer.print(con);

		 for(i=0; i < l; i++){

		     tempEdge = (edgeParameter) Alld.get(i);
		     String temp = new String(tempEdge.getEdgeID());
		     int t = tempEdge.getTypeFlag();
		     int temflag;
		     printer.print("<edge id=\""+temp);		     

		     if(t == 1){
			  printer.print("\" fromnode=\"");

			  String temp1 = new String(tempEdge.getFromnode());
			  printer.print(temp1+"\" tonode=\"");

			  String temp2 = new String(tempEdge.getTonode());
			  printer.print(temp2+"\"");
		     }
		     else if(t ==2){
			  printer.print("\" xfrom=\"");
			  String temp3 = new String(tempEdge.getXfrom());
			  printer.print(temp3+"\" yfrom=\"");
			  String temp4 = new String(tempEdge.getYfrom());
			  printer.print(temp4+"\" xto=\"");
			  String temp5 = new String(tempEdge.getXto());
			  printer.print(temp5+"\" yto=\"");
			  String temp6 = new String(tempEdge.getYto());
			  printer.print(temp6+"\"");
		     }
		     temflag = tempEdge.getTypeIDFlag();
		     if(temflag == 1){
		    	 String temp7 = new String(tempEdge.getTypeID());
			printer.print(" type=\""+temp7+"\"");
		     }
		     else{

			temflag = tempEdge.getPriorityFlag();
			if(temflag == 1){				
			    temp = new String(tempEdge.getPriority());
			    printer.print(" priority=\""+temp+"\"");
			}
			temflag = tempEdge.getNolanesFlag();
			if(temflag == 1){
			    temp = new String(tempEdge.getNolanes());
			    printer.print(" nolanes=\""+temp+"\"");
			}
			temflag = tempEdge.getSpeedFlag();
			if(temflag == 1){
			    temp = new String(tempEdge.getSpeed());
			    printer.print(" speed=\""+temp+"\"");
			}
			temflag = tempEdge.getLengthFlag();
			if(temflag == 1){
			    temp = new String(tempEdge.getLength());
			    printer.print(" length=\""+temp+"\"");
			}
		     }
		     temflag = tempEdge.getShapeFlag();
		     if(temflag == 1){
			    temp = new String(tempEdge.getShape());
			    printer.print(" shape=\""+temp+"\"");
		     }
		     temflag = tempEdge.getSpreadFlag();
		     if(temflag == 1){
			    temp = new String(tempEdge.getSpread());
			    printer.print(" spread_type=\""+temp+"\"");
		     }

		     printer.print("/>\n");
		 }
		 printer.print("</edges>\n");
		 printer.close();
	     }
}




