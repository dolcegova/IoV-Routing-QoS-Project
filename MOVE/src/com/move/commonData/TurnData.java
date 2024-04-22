package com.move.commonData;
/**
   Thesis implementation
   Class : turnData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class TurnData{

      ArrayList allTurns;
      boolean empty;

      public TurnData(){
          allTurns = new ArrayList();
      }

      public ArrayList returnData(){
          return allTurns;
      }
      
      public void addIntervals(TurnInterval interval){
          allTurns.add(interval);
      }

      public void clearNodes(){
          allTurns.clear();
      }

      public boolean isEmpty(){
          empty=allTurns.isEmpty();
          return empty;
      }

      public void deleteType(int index){
           allTurns.remove(index);
      }


}
