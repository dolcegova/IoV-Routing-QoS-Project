package com.move.commonData;
/**
   Thesis implementation
   Class : edgeData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

import java.util.*;

public class edgeData{

      ArrayList allEdges;
      boolean empty;

      public edgeData(){
          allEdges = new ArrayList();
      }

      public ArrayList returnData(){
          return allEdges;
      }

      public void addEdges(edgeParameter edge){
          allEdges.add(edge);
      }

      public void clearNodes(){
          allEdges.clear();
      }

      public boolean isEmpty(){
          empty=allEdges.isEmpty();
          return empty;
      }

      public void deleteEdge(int index){
           allEdges.remove(index);
      }


}






