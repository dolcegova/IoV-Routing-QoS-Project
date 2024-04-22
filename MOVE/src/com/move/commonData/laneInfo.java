package com.move.commonData;
/**
   Thesis implementation
   Class : laneInfo.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

import java.util.*;

public class laneInfo{

      ArrayList allvehs;
      String id="";
      boolean empty;

      public laneInfo(String identification){
          allvehs = new ArrayList();
          id = identification;
      }

      public String getID(){
        return id;
      }

      public ArrayList returnData(){
          return allvehs;
      }

      public void addVehs(vehInfo veh){
          allvehs.add(veh);
      }

      public void clearVehs(){
          allvehs.clear();
      }

      public boolean isEmpty(){
          empty=allvehs.isEmpty();
          return empty;
      }

      public void deleteVeh(int index){
           allvehs.remove(index);
      }

}

