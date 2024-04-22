package com.move.commonData;
/**
   Thesis implementation
   Class : stepInfo.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class stepInfo{

      ArrayList alledges;
      String id="";
      boolean empty;

      public stepInfo(String identification){
          alledges = new ArrayList();
          id = identification;
      }

      public String getID(){
        return id;
      }

      public ArrayList returnData(){
          return alledges;
      }

      public void addEdges(edgeInfo edge){
          alledges.add(edge);
      }

      public void clearEdges(){
          alledges.clear();
      }

      public boolean isEmpty(){
          empty=alledges.isEmpty();
          return empty;
      }

      public void deleteEdge(int index){
           alledges.remove(index);
      }

}

