package com.move.commonData;
/**
   Thesis implementation
   Class : edgeInfo.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

import java.util.*;

public class edgeInfo{

      ArrayList allLanes;
      String id="";
      boolean empty;

      public edgeInfo(String identification){
          allLanes = new ArrayList();
          id = identification;
      }

      public String getID(){
        return id;
      }

      public ArrayList returnData(){
          return allLanes;
      }

      public void addLanes(laneInfo lane){
          allLanes.add(lane);
      }

      public void clearLanes(){
          allLanes.clear();
      }

      public boolean isEmpty(){
          empty=allLanes.isEmpty();
          return empty;
      }

      public void deleteLane(int index){
           allLanes.remove(index);
      }

}

