package com.move.commonData;
/**
   Thesis implementation
   Class : dumpData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

import java.util.*;

public class dumpData{

      ArrayList allSteps;

      boolean empty;

      public dumpData(){
          allSteps = new ArrayList();

      }

      public ArrayList returnData(){
          return allSteps;
      }

      public void addSteps(stepInfo step){
          allSteps.add(step);
      }

      public void clearSteps(){
          allSteps.clear();
      }

      public boolean isEmpty(){
          empty=allSteps.isEmpty();
          return empty;
      }

      public void deleteStep(int index){
           allSteps.remove(index);
      }

}

