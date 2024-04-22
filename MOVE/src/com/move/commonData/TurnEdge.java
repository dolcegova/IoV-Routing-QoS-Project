package com.move.commonData;
/**
   Thesis implementation
   Class : turnData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class TurnEdge{

      ArrayList allEdge;
      boolean empty;

      String id;
      
      public TurnEdge(String iden){
	  allEdge = new ArrayList();
	  id = iden;
      }
      
      public String getID(){
	  return id;
      }
      
      public ArrayList returnData(){
	  return allEdge;
      }
      
      public void addParameters(TurnParameter turn){
	  allEdge.add(turn);
      }

      public void clearNodes(){
	  allEdge.clear();
      }

      public boolean isEmpty(){
	  empty=allEdge.isEmpty();
	  return empty;
      }

      public void deleteType(int index){
	   allEdge.remove(index);
      }


}
