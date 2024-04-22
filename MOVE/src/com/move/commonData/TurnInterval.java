package com.move.commonData;
/**
   Thesis implementation
   Class : turnInterval.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class TurnInterval{

      ArrayList allInterval;
      boolean empty;
      
      String begin;
      String end;
      
      public TurnInterval(String b,String e){
	  allInterval = new ArrayList();
	  begin = b;
	  end = e;
      }

      public String getBegin(){
	  return begin;
      }
      
      public String getEnd(){
	  return end;
      }
      
      public ArrayList returnData(){
	   return allInterval;
      }
      
      public void addEdge(TurnEdge edge){
	  allInterval.add(edge);
      }

      public void clearNodes(){
	  allInterval.clear();
      }

      public boolean isEmpty(){
	  empty=allInterval.isEmpty();
	  return empty;
      }

      public void deleteType(int index){
	   allInterval.remove(index);
      }


}
