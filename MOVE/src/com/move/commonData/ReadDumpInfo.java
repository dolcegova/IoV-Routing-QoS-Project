package com.move.commonData;
// This file handles SUMO's trace file. 

import java.io.*;
import java.util.*;


public class ReadDumpInfo{

       String filename = "";
       dumpData data;


       public ReadDumpInfo(String name){
            filename = name;
            data = new dumpData();
       }

       public dumpData getData(){
         return data;
       }

       public int countToken(String s){
            int result = 0;
            for(int i =0; i<s.length();i++){
                if(s.charAt(i) == '=')
                  result++;
            }
            return (result+1);
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

       public String getValue(String token){
            int i = token.length();
            int start=0;
            int end = 0;
            int no=0;

            for(int j=0;j<i;j++){
              if((token.charAt(j) == '\"')&&(no==0)){
                  no++;
                  start = j;
              }
              else if((token.charAt(j) == '\"')&&(no==1)){
                  end = j;
                  break;
              }
            }
            return (token.substring(start+1,end));
       }

       public String getValueName(String token){
            int i = token.length();
            int end = 0;
            for(int j=0;j<i;j++){
              if(token.charAt(j) == '='){

                  end = j;
                  break;
              }
            }
            return (token.substring(0,end));
       }

       public void LoadInfo() throws Exception{

        stepInfo temStep = new stepInfo("");
        edgeInfo temEdge = new edgeInfo("");
        laneInfo temLane = new laneInfo("");
        vehInfo  temVeh = new vehInfo("","","");

        String timeID = "";
        String edgeID = "";
        String laneID = "";
        String vehicleID = "";

        String pos="";
        String speed="";

        File filterfile = new File(filename);
        BufferedReader getContent = new BufferedReader(new FileReader(filterfile));
        StringBuffer tmpcontent = new StringBuffer();

        String line = getContent.readLine();

        while(line!=null){

          tmpcontent = new StringBuffer();

          int x=0;
          int argumentNO = line.length();
          while (x<argumentNO){
              tmpcontent.append(line.charAt(x));
              x++;
          }
          tmpcontent.append('\n');
          line = clearSpace(line);

          //
          if(line.length()>=7){

           StringTokenizer st= new StringTokenizer(line);

           String temp = st.nextToken();

           int arglength = countToken(line);
           int i = 0;

          //System.out.println(line);
          while(i < arglength){

            //System.out.println(temp+arglength);
           if(temp.compareTo("<timestep")==0){
               temp = st.nextToken();
               timeID = getValue(temp);
               temStep = new stepInfo(timeID);
               i++;
           }
           else if(temp.compareTo("<edge")==0){
               temp = st.nextToken();
               edgeID = getValue(temp);
               temEdge = new edgeInfo(edgeID);
               i++;
           }
           else if(temp.compareTo("<lane")==0){
               temp = st.nextToken();
               laneID = getValue(temp);
               temLane = new laneInfo(laneID);
               i++;

           }
           else if(temp.compareTo("<vehicle")==0){
               temp = st.nextToken();
               vehicleID = getValue(temp);
               temp = st.nextToken();
               i++;
           }
           else if(temp.compareTo("</lane>")==0){
               temEdge.addLanes(temLane);
           }
           else if(temp.compareTo("</edge>")==0){
               temStep.addEdges(temEdge);
           }
           else if(temp.compareTo("</timestep>")==0){
               data.addSteps(temStep);
           }
           else{
               String valueName = getValueName(temp);
               //System.out.println(temp+"  "+valueName+ "   "+i + "  " +arglength);
               if(valueName.compareTo("pos")==0){
                   //System.out.println("pos"+ "   "+i+ "  " +arglength);

                   pos = getValue(temp);
                   temp = st.nextToken();
                   //System.out.println(temp);
               }
               else if(valueName.compareTo("speed")==0){
                   //System.out.println("speed");

                   speed = getValue(temp);
                   temVeh = new vehInfo(vehicleID,pos,speed);
                   temLane.addVehs(temVeh);
               }
           }

           i++;
          }
          }
          line = getContent.readLine();
        }
        
       }
      
}







