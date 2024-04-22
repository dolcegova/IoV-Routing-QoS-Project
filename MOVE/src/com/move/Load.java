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



public class Load {
          String filename;
          String filetype;
          String simulationtype;

          static int fullpath = 0;

          /**********************
           * Store results
           *********************/
           edgeData edgeList;
           nodeData nodeList;
           typeData typeList;
           flowData flowList;
           tripData tripList;
           TurnData turnList;
           routeData routeList;

           String cfgtype      ="null";
           String cfglanenumber="null";
           String cfgspeed     ="null";
           String cfgpriority  ="null";

           String sumobegin = "null";
           String sumoend   = "null";

          public Load(String target){
          	fullpath = 1;
          	filename = target;
          }
	  
          public Load(String f,String t1, String t2){
            filename = f;
            simulationtype = t1;
            filetype = t2;

          }

	public void initNodes(){
		nodeList = new nodeData();
	}

	public void initEdges(){
		edgeList = new edgeData();
	}

	public void initTypes(){
		typeList = new typeData();
	}

	public void initFlows(){
		flowList = new flowData();
	}
	
	public void initTrips(){
		tripList = new tripData();
	}
	
	public void initTurns(){
		turnList = new TurnData();
	}
	
	public void initRoutes(){
		 routeList = new routeData();
	}

          public int returnflag(){
            int dataFlag = 0;

            if(simulationtype.compareTo("nod")==0)
                dataFlag = 1;
            else if(simulationtype.compareTo("edg")==0)
                dataFlag = 2;
            else if(simulationtype.compareTo("typ")==0)
                dataFlag = 3;
            else if(simulationtype.compareTo("netc")==0)
                dataFlag = 4;
            else if(simulationtype.compareTo("sumo")==0)
                dataFlag = 5;
            else if(simulationtype.compareTo("flowdefs")==0)
                dataFlag = 6;
            else if(simulationtype.compareTo("tripdefs")==0)
                dataFlag = 7;
            else if(simulationtype.compareTo("turndefs")==0)
                dataFlag = 8;
            else if(simulationtype.compareTo("rou")==0)
                dataFlag = 9;
            return dataFlag;
          }

          public routeData getRouteData(){
            return (routeList);
          }

          public TurnData getTurnData(){
            return (turnList);
          }

          public tripData getTripData(){
            return (tripList);
          }

          public flowData getFlowData(){
            return (flowList);
          }

          public String getsumobegin(){

               return (sumobegin);
          }

          public String getsumoend(){

               return (sumoend);
          }

          public String getcfgtype(){

               return (cfgtype);
          }

          public String getcfglanenumber(){

               return (cfglanenumber);
          }

          public String getcfgspeed(){

               return (cfgspeed);
          }

          public String getcfgpriority(){

               return (cfgpriority);
          }

          public nodeData getNodeData(){

               return (nodeList);
          }

          public edgeData getEdgeData(){

               return (edgeList);
          }

          public typeData getTypeData(){
               return (typeList);
          }

          public String clearSpace(String l){
            String result = "";
            int length = l.length();
            int start = 0;

            while(start < length){
               if((l.charAt(start)=='<')||(l.charAt(start)=='\n')){
                 break;
               }
               else
                 start++;
            }
            return (l.substring(start));
          }

          public String getValue(String l){
            int length = l.length();
            int start=0;
            int end=0;

            while(start < length){
              if((l.charAt(start)=='\"')&&(start!=0)){
                end = start;
                break;
              }
              else if(l.charAt(start)=='\n'){
                end = start;
                break;
              }
              else
                start++;

            }
            return (l.substring(1,end));
          }

          public int countToken(String s){
            int result = 0;
            for(int i =0; i<s.length();i++){
                if(s.charAt(i) == '=')
                  result++;
            }
            return (result+1);
          }

          //check to see if this token is a complete definition
          public int checkEnd(String s){
            int number = 0;
            for(int i=0; i<s.length(); i++){
              if(s.charAt(i) == '\"'){
                number++;
              }
            }
            if(s.charAt(0) == '<'){
              return 1;
            }
            else if(number ==2)
              return 1;
            else if((number ==1)||(number == 0)){
              return 0;
            }

            return 1000; //return 1000 if something wrong

          }

          public int checkStart(String s){
            for(int i=0; i<s.length();i++){
              if(s.charAt(i)=='='){
                return 1;
              }
            }
            return 0;
          }
          //find the starting position of this token
          public int findPosition(String sentence,int index){

            StringTokenizer st= new StringTokenizer(sentence);

            String[] tempList = sentence.split(" ");
            int result = 1;
            int bool = 0;
            int i = 1;
            while(i < st.countTokens()){

                bool = checkStart(tempList[i]);
                if(bool == 1){
                  if(result == index)
                    return i;
                  else{
                    result++;
                    i++;
                  }
                }
                else
                   i++;
            }
            System.out.println("Wrong index");

            return 0;
          }
          //Return the whole definition, p is in decrease order
          public String findToken(String n,int p){
            int t = countToken(n);
            int position = t-p;
            int index=0;
            StringTokenizer st= new StringTokenizer(n);

            String[] tokens = n.split(" ");

            if(position == 0)
              return tokens[0];

            int pos = findPosition(n,position);
            String result = tokens[pos];

            //String result = tokens[position];
            int isFull = checkEnd(result);

            //System.out.println("outside: "+result+"index: "+position);

            if(isFull == 1){
              //System.out.println("First if "+tokens[pos]);
              return (tokens[pos]);
            }
            else if(isFull == 0){
              int isStart = 10;
              isStart = checkStart(result);
              if(isStart == 1){
                while (isFull == 0){
                 //System.out.println("while: "+result+"index: "+position);
                 pos++;
                 result = result.concat(" "+tokens[pos]);
                 isFull = checkEnd(result);
                }
                //System.out.println("Second if "+result);
                return result;
              }
              else{
                //int pos = findPosition(n,position);


                result = tokens[pos];
                while (isFull == 0){
                 pos++;
                 result = result.concat(tokens[pos]);
                 isFull = checkEnd(result);
                }
                //System.out.println("else "+result);
                return result;
              }

            }
            System.out.println("Wrong Token");
            return result;

          }

          public String findType(String nline){
            int x=0;
            int argumentNO = nline.length();

            StringBuffer result = new StringBuffer();

            while (x<argumentNO){
               if(nline.charAt(x)=='<'){
                  x++;
                  while (nline.charAt(x) != '>'){
                     result.append(nline.charAt(x));
                     x++;
                  }
               }
               break;
            }
            return result.toString();

          }

          public String findValue(String nline){
            int x=0;
            int argumentNO = nline.length();

            StringBuffer result = new StringBuffer();

            while (x<argumentNO){
               if(nline.charAt(x)=='>'){
                  x++;
                  while (nline.charAt(x) != '<'){
                     result.append(nline.charAt(x));
                     x++;
                  }
                  break;
               }
               x++;
            }
            return result.toString();
          }

          public String extractInfo(String nline){
            int x=0;
            int argumentNO = nline.length();

            StringBuffer result = new StringBuffer();

            while (x<argumentNO){
               if(nline.charAt(x)=='<'){
                  result.append(nline.charAt(x));
                  x++;
                  while (nline.charAt(x) != '>'){
                     result.append(nline.charAt(x));
                     x++;
                  }
                  result.append(nline.charAt(x));
                  break;
               }
               x++;
            }
            return result.toString();
          }

          public String findRouteInfo(String line, int number){               
               StringBuffer result = new StringBuffer();

               if(number == 1){
                 int i = 0;
                 while( i<line.length()){
                   if(line.charAt(i) == '\"'){
                     i++;
                     for(int j=i;j<line.length();j++){
                        if(line.charAt(j) == '\"'){
                          return result.toString();
                        }
                        else{
                          result.append(line.charAt(j));
                        }
                     }
                   }
                   i++;
                 }
               }
               else if(number == 2){
                 int count=0;

                 for(int i = 0;i<line.length();i++){
                   if(line.charAt(i)=='\"'){
                     count++;
                   }

                   if(count == 3){
                     i++;
                     while(line.charAt(i)!='\"'){
                       result.append(line.charAt(i));
                       i++;
                     }
                     return result.toString();
                   }
                 }
               }
               else if(number == 3){
                 int i = 0;
                 while(i<line.length()){
                   if(line.charAt(i) == '>'){
                     i++;
                     for(int j=i;j<line.length();j++){
                        if(line.charAt(j) == '<'){
                          return result.toString();
                        }
                        else{
                          result.append(line.charAt(j));
                        }
                     }
                   }
                   i++;
                 }
               }
               return "null";
          }





          public void readfile() throws Exception{
            String fullname;
               if (fullpath == 1)
                    fullname = filename;
                else            
                    fullname = filename + "." + simulationtype + "." + filetype;     
            File filterfile = new File(fullname);
            BufferedReader getContent = new BufferedReader(new FileReader(filterfile));
            StringBuffer tmpcontent = new StringBuffer();

            String line = "";


            line = getContent.readLine();

            if(line.compareTo("<edges>")==0){

              line = getContent.readLine();
              while(line != null){

                        String edgeID="";

                        String fromnode="";
                        String tonode="";

                        String priority="";
                        int priorityflag = 0;

                        String nolanes="";
                        int nolanesflag = 0;

                        String speed="";
                        int speedflag = 0;

                        String length="";
                        int lengthflag = 0;

                        String typeID="";
                        int typeIDflag=0;

                        String shape="";
                        int shapeFlag=0;

                        String spread_type="";
                        int spread_typeFlag=0;

                        String xfrom="";
                        String yfrom="";
                        String xto="";
                        String yto="";


                        tmpcontent = new StringBuffer();

                        int x=0;
                        int argumentNO = line.length();

                        while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                        }
                        tmpcontent.append('\n');
                        String temline = tmpcontent.toString();
                        String nline = clearSpace(temline);

                        //StringTokenizer st= new StringTokenizer(nline);
                        int arglength = countToken(nline);


                        int flag = 0;
                        while(arglength != 0){
                            String check = findToken(nline,arglength);

                            if(check.compareTo("<edge") == 0){
                              arglength--;
                            }
                            else{
                               //System.out.println("else"+" "+check);
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 edgeID = getValue(tem);

                               }
                               else if(tem.compareTo("fromnode")==0){
                                 tem = split[1];
                                 fromnode = getValue(tem);
                                 flag = 1;
                               }
                               else if(tem.compareTo("tonode")==0){
                                 tem = split[1];
                                 tonode = getValue(tem);
                               }
                               else if(tem.compareTo("xfrom") == 0){
                                 tem = split[1];
                                 xfrom = getValue(tem);
                                 flag = 2;
                               }
                               else if(tem.compareTo("yfrom")==0){
                                 tem = split[1];
                                 yfrom = getValue(tem);
                               }
                               else if(tem.compareTo("xto")==0){
                                 tem = split[1];
                                 xto = getValue(tem);
                               }
                               else if(tem.compareTo("yto")==0){
                                 tem = split[1];
                                 yto = getValue(tem);
                               }
                               else if(tem.compareTo("priority")==0){
                                 tem = split[1];
                                 priority = getValue(tem);
                                 priorityflag = 1;
                               }
                               else if(tem.compareTo("nolanes")==0){
                                 tem = split[1];
                                 nolanes = getValue(tem);
                                 nolanesflag = 1;
                               }
                               else if(tem.compareTo("speed")==0){
                                 tem = split[1];
                                 speed = getValue(tem);
                                 speedflag = 1;
                               }
                               else if(tem.compareTo("length")==0){
                                 tem = split[1];
                                 length = getValue(tem);
                                 lengthflag = 1;
                               }
                               else if(tem.compareTo("type") == 0){
                                 tem = split[1];
                                 typeID = getValue(tem);
                                 typeIDflag = 1;
                               }
                               else if(tem.compareTo("shape")==0){
                                 tem = split[1];
                                 shape = getValue(tem);
                                 shapeFlag = 1;
                               }
                               else if(tem.compareTo("spread_type")==0){
                                 tem = split[1];
                                 spread_type = getValue(tem);
                                 spread_typeFlag = 1;
                               }
                            }
                        }
                        if(nline.length()>15){
                          if(flag == 1){
                                 edgeParameter edge = new edgeParameter(edgeID,fromnode,tonode);

                                 if(priorityflag ==1)
                                      edge.setPriority(priority);

                                 if(nolanesflag == 1)
                                      edge.setNolanes(nolanes);

                                 if(speedflag == 1)
                                      edge.setSpeed(speed);

                                 if(lengthflag == 1)
                                      edge.setLength(length);

                                 if(typeIDflag == 1)
                                      edge.setType(typeID);

                                 if(shapeFlag == 1)
                                     edge.setShape(shape);

                                 if(spread_typeFlag == 1)
                                     edge.setSpread(spread_type);

                                 edgeList.addEdges(edge);
                          }
//                          else if(flag == 2){
//                                 edgeParameter edge = new edgeParameter(edgeID,xfrom,yfrom,xto,yto);
//
//                                 if(priorityflag ==1)
//                                      edge.setPriority(priority);
//
//                                 if(nolanesflag == 1)
//                                      edge.setNolanes(nolanes);
//
//                                 if(speedflag == 1)
//                                      edge.setSpeed(speed);
//
//                                 if(lengthflag == 1)
//                                      edge.setLength(length);
//
//                                 if(typeIDflag == 1)
//                                      edge.setType(typeID);
//
//                                 if(shapeFlag == 1)
//                                     edge.setShape(shape);
//
//                                 if(spread_typeFlag == 1)
//                                     edge.setSpread(spread_type);
//                          }
                        }
                        line = getContent.readLine();
              }
              //String content = tmpcontent.toString();
              //System.out.println(content);
            }
            else if(line.compareTo("<nodes>")==0){
              line = getContent.readLine();
              while(line != null){
                 String id="null";
                 String X="null";
                 String Y="null";
                 String type="null";


                 tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();

                 String nline = clearSpace(temline);

                 StringTokenizer st= new StringTokenizer(nline);
                 int arglength = st.countTokens();

                 while(arglength != 0){

                            String check = st.nextToken();

                            if(check.compareTo("<node") == 0){
                               arglength--;
                            }
                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);

                               }
                               else if(tem.compareTo("x")==0){
                                 tem = split[1];
                                 X = getValue(tem);
                               }
                               else if(tem.compareTo("y")==0){
                                 tem = split[1];
                                 Y = getValue(tem);
                               }
                               else if(tem.compareTo("type")==0){
                                 tem = split[1];
                                 type = getValue(tem);
                               }
                            }
                 }
                 if(nline.length()>15){
                  nodeParameter node = new nodeParameter(id,X,Y,type);
                  nodeList.addNodes(node);
                 }
                 line = getContent.readLine();
              }
            }


            else if(line.compareTo("<types>")==0){
              line = getContent.readLine();
              while(line != null){
                 String id="";
                 String name="";
                 String nolanes="";
                 String speed="";
                 String priority="";
                 String capacity="";
                 String parkingPossible="";

                 tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();

                 String nline = clearSpace(temline);

                 //StringTokenizer st= new StringTokenizer(nline);
                 int arglength = countToken(nline);

                 //System.out.println(arglength);

                 while(arglength != 0){

                            String check = findToken(nline,arglength);

                            //System.out.println(check);

                            if(check.compareTo("<type") == 0){
                               arglength--;
                            }
                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);

                               }
                               else if(tem.compareTo("name")==0){
                                 tem = split[1];
                                 //System.out.println(tem);
                                 name = getValue(tem);
                               }
                               else if(tem.compareTo("nolanes")==0){
                                 tem = split[1];
                                 nolanes = getValue(tem);
                               }
                               else if(tem.compareTo("priority")==0){
                                 tem = split[1];
                                 priority = getValue(tem);
                               }
                               else if(tem.compareTo("speed")==0){
                                 tem = split[1];
                                 speed = getValue(tem);
                               }
                               else if(tem.compareTo("Capacity")==0){
                                 tem = split[1];
                                 capacity = getValue(tem);

                               }
                               else if(tem.compareTo("ParkingPossible")==0){
                                 tem = split[1];
                                 parkingPossible = getValue(tem);
                               }
                            }
                 }
                 if(nline.length()>15){
                  typeParameter type = new typeParameter(id,name,nolanes,speed,priority,capacity,parkingPossible);
                  typeList.addTypes(type);
                 }
                 line = getContent.readLine();
              }
            }

            else if(line.compareTo("<configuration>")==0){
              line = getContent.readLine();
              while(line != null){
                tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();
                 String nline = clearSpace(temline);
                 String checkType = findType(nline);

                 if(checkType.compareTo("type") == 0){
                         cfgtype =  findValue(nline);
                 }
                 else if(checkType.compareTo("lanenumber")==0){
                         cfglanenumber = findValue(nline);
                 }
                 else if(checkType.compareTo("speed")==0){
                         cfgspeed = findValue(nline);
                 }
                 else if(checkType.compareTo("priority")==0){
                         cfgpriority = findValue(nline);
                 }
                 else if(checkType.compareTo("begin")==0){
                         sumobegin = findValue(nline);
                 }
                 else if(checkType.compareTo("end")==0){
                         sumoend = findValue(nline);
                 }
                 line = getContent.readLine();
              }
            }

            else if(line.compareTo("<flows>")==0){
              line = getContent.readLine();
              while(line != null){
                String id = "null";
                String from = "null";
                String to = "null";
                String begin = "null";
                String end = "null";
                String no = "null";

                tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();

                 String nline = clearSpace(temline);

                 int arglength = countToken(nline);

                 while(arglength != 0){

                            String check = findToken(nline,arglength);

                            //System.out.println(check);

                            if(check.compareTo("<flow") == 0){
                               arglength--;
                            }
                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);

                               }
                               else if(tem.compareTo("from")==0){
                                 tem = split[1];
                                 from = getValue(tem);
                               }
                               else if(tem.compareTo("to")==0){
                                 tem = split[1];
                                 to = getValue(tem);
                               }
                               else if(tem.compareTo("begin")==0){
                                 tem = split[1];
                                 begin = getValue(tem);
                               }
                               else if(tem.compareTo("end")==0){
                                 tem = split[1];
                                 end = getValue(tem);
                               }
                               else if(tem.compareTo("no")==0){
                                 tem = split[1];
                                 no = getValue(tem);
                               }
                            }

                 }
                 if(nline.length()>15){
                  flowParameter flow = new flowParameter(id,from,to,begin,end,no);
                  flowList.addFlows(flow);
                 }
                 line = getContent.readLine();
              }
            }
            else if(line.compareTo("<tripdefs>")==0){
              line = getContent.readLine();
              while(line != null){
                String id     ="";
                String depart ="";
                String from   ="";
                String to     ="";
                String type   ="";
                String period ="";
                String repno  ="";

                tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();

                 String nline = clearSpace(temline);

                 int arglength = countToken(nline);

                 while(arglength != 0){

                            String check = findToken(nline,arglength);

                            //System.out.println(check);

                            if(check.compareTo("<tripdef") == 0){
                               arglength--;
                            }
                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);

                               }
                               else if(tem.compareTo("depart")==0){
                                 tem = split[1];
                                 depart = getValue(tem);
                               }
                               else if(tem.compareTo("from")==0){
                                 tem = split[1];
                                 from = getValue(tem);
                               }
                               else if(tem.compareTo("to")==0){
                                 tem = split[1];
                                 to = getValue(tem);
                               }
                               else if(tem.compareTo("type")==0){
                                 tem = split[1];
                                 type = getValue(tem);
                               }
                               else if(tem.compareTo("period")==0){
                                 tem = split[1];
                                 period = getValue(tem);
                               }
                               else if(tem.compareTo("repno")==0){
                                 tem = split[1];
                                 repno = getValue(tem);
                               }
                            }

                 }
                 if(nline.length()>15){
                  tripParameter trip = new tripParameter(id,depart,from,to,period,type,repno);
                  tripList.addTrips(trip);
                 }
                 line = getContent.readLine();
              }
            }
            else if(line.compareTo("<turn-defs>")==0){
              line = getContent.readLine();
              TurnInterval newInt = new TurnInterval("null","null");

              while(line.compareTo("</turn-defs>") != 0){
                if(line.compareTo("</interval>") == 0){
                    turnList.addIntervals(newInt);
                    line = getContent.readLine();
                    line = extractInfo(line);
                }
                else{

                  String begin = "";
                  String end   = "";

                  tmpcontent = new StringBuffer();

                  int x=0;
                  int argumentNO = line.length();

                  while (x<argumentNO){
                         tmpcontent.append(line.charAt(x));
                         x++;
                  }
                  tmpcontent.append('\n');
                  String temline = tmpcontent.toString();
                  String nline = clearSpace(temline);
                  int arglength = countToken(nline);

                  while(arglength != 0){

                            String check = findToken(nline,arglength);

                            if(check.compareTo("<interval") == 0){
                               arglength--;
                            }
                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("begin") == 0){
                                 tem = split[1];
                                 begin = getValue(tem);
                               }
                               else if(tem.compareTo("end") == 0){
                                 tem = split[1];
                                 end = getValue(tem);
                               }
                            }
                  }
                  newInt = new TurnInterval(begin,end);
                  line = getContent.readLine();
                  TurnEdge turnedges = new TurnEdge("null");

                  while(line.compareTo("</interval>") != 0){


                    if(line.compareTo("</fromedge>") == 0){
                       newInt.addEdge(turnedges);
                       line = getContent.readLine();
                       line = extractInfo(line);
                    }
                    else{

                     String id = "";

                     tmpcontent = new StringBuffer();

                     int m=0;
                     int argumentNO1 = line.length();

                     while (m<argumentNO1){
                         tmpcontent.append(line.charAt(m));
                         m++;
                     }
                     tmpcontent.append('\n');
                     String temline1 = tmpcontent.toString();
                     String nline1 = clearSpace(temline1);
                     int arglength1 = countToken(nline1);

                     while(arglength1 != 0){

                            String check = findToken(nline1,arglength1);

                            if(check.compareTo("<fromedge") == 0){
                               arglength1--;
                            }
                            else{
                               arglength1--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);
                               }
                            }
                     }
                     turnedges = new TurnEdge(id);
                     line = getContent.readLine();
                     TurnParameter turnparam;



                     while(line.compareTo("</fromedge>") != 0){
                         //System.out.println(line+" "+line.length());

                         String id1 = "";
                         String perc = "";

                         tmpcontent = new StringBuffer();

                         int n=0;
                         int argumentNO2 = line.length();

                         while (n<argumentNO2){
                           tmpcontent.append(line.charAt(n));
                           n++;
                         }
                         tmpcontent.append('\n');
                         String temline2 = tmpcontent.toString();
                         String nline2 = clearSpace(temline2);
                         int arglength2 = countToken(nline2);

                         while(arglength2 != 0){

                            String check = findToken(nline2,arglength2);

                            if(check.compareTo("<toedge") == 0){
                               arglength2--;
                            }
                            else{
                               arglength2--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;
                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id1 = getValue(tem);
                               }
                               else if(tem.compareTo("perc")==0){
                                 tem = split[1];
                                 perc = getValue(tem);
                               }
                               else if(tem.compareTo("probability")==0){
                                   tem = split[1];
                                   perc = getValue(tem);
                               }
                            }
                         }

                       turnparam = new TurnParameter(id1,perc);
                       turnedges.addParameters(turnparam);
                       line = getContent.readLine();
                       line = extractInfo(line);
                     }
                    }

                  }
                }

                //line = getContent.readLine();
              }
            }

            else if(line.compareTo("<routes>")==0){
              line = getContent.readLine();
              while(line != null){
                String id = "";
                String accel = "";
                String decel = "";
                String sigma = "";
                String length = "";
                String maxspeed = "";
                String type = "";
                String route = "";
                String depart = "";
                String period = "";
                String repno = "";

                String multi_ref = "";
                String path = "";

                tmpcontent = new StringBuffer();

                 int x=0;
                 int argumentNO = line.length();

                 while (x<argumentNO){
                             tmpcontent.append(line.charAt(x));
                             x++;
                 }
                 tmpcontent.append('\n');
                 String temline = tmpcontent.toString();

                 String nline = clearSpace(temline);

                 int arglength = countToken(nline);
                 int typeFlag = 0;

                 if(arglength != 3){
                    while(arglength != 0){

                            String check = findToken(nline,arglength);

                            if(check.compareTo("<vtype") == 0){
                               arglength--;
                               typeFlag = 1;
                            }
                            else if(check.compareTo("<vehicle") == 0){
                               arglength--;
                               typeFlag = 2;
                            }

                            else{
                               arglength--;
                               String[] split = check.split("=");
                               String tem = split[0];
                               String value;

                               if(tem.compareTo("id") == 0){
                                 tem = split[1];
                                 id = getValue(tem);

                               }
                               else if(tem.compareTo("accel")==0){
                                 tem = split[1];
                                 accel = getValue(tem);
                               }
                               else if(tem.compareTo("decel")==0){
                                 tem = split[1];
                                 decel = getValue(tem);
                               }
                               else if(tem.compareTo("sigma")==0){
                                 tem = split[1];
                                 sigma = getValue(tem);
                               }
                               else if(tem.compareTo("length")==0){
                                 tem = split[1];
                                 length = getValue(tem);

                               }
                               else if(tem.compareTo("maxspeed")==0){
                                 tem = split[1];
                                 maxspeed = getValue(tem);
                               }
                               else if(tem.compareTo("type")==0){
                                 tem = split[1];
                                 type = getValue(tem);
                               }
                               else if(tem.compareTo("route")==0){
                                 tem = split[1];
                                 route = getValue(tem);
                               }
                               else if(tem.compareTo("depart")==0){
                                 tem = split[1];
                                 depart = getValue(tem);
                               }
                               else if(tem.compareTo("period")==0){
                                 tem = split[1];
                                 period = getValue(tem);
                               }
                               else if(tem.compareTo("repno")==0){
                                 tem = split[1];
                                 repno = getValue(tem);
                               }
                            }
                    }
                 }
                 else if(arglength == 3){

                           id = findRouteInfo(nline,1);
                           multi_ref = findRouteInfo(nline,2);
                           path = findRouteInfo(nline,3);
                           arglength = 0;
                 }

                 if(nline.length()>15){
                  if(typeFlag == 1){
                      vtypeParameter vt = new vtypeParameter(id, accel, decel, sigma, length, maxspeed);
                      routeList.addTypes(vt);
                  }
                  else if(typeFlag == 2){
                      vehicleParameter veh = new vehicleParameter(id, route, type, depart, period, repno);
                      routeList.addVehicles(veh);
                  }
                  else{
                      routeParameter rou = new routeParameter(id,multi_ref,path);
                      routeList.addRoutes(rou);
                  }
                 }
                 line = getContent.readLine();
              }

            }

          }




}


