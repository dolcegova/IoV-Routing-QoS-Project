package com.move.commonData;
/**
   Thesis implementation
   Class : flowData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class flowData{

      ArrayList allFlows;
      boolean empty;

      public flowData(){
          allFlows = new ArrayList();
      }

      public ArrayList returnData(){
          return allFlows;
      }
      
      public void addFlows(flowParameter flow){
          allFlows.add(flow);
      }

      public void clearFlows(){
          allFlows.clear();
      }

      public boolean isEmpty(){
          empty=allFlows.isEmpty();
          return empty;
      }

      public void deleteFlow(int index){
           allFlows.remove(index);
      }


}































