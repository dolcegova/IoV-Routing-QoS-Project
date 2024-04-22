package com.move.commonData;
public class nodeInfo{
       private String nodeID = "";
       private String xPosition = "";
       private String yPosition = "";

       public nodeInfo(String id, String xpos, String ypos){
           nodeID = id;
           xPosition = xpos;
           yPosition = ypos;
       }

       public String getID(){
          return nodeID;
       }

       public String getX(){
         return xPosition;
       }

       public String getY(){
         return yPosition;
       }
}







