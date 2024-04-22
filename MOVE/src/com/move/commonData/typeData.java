package com.move.commonData;
/**
   Thesis implementation
   Class : typeData.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


import java.util.*;

public class typeData{

      ArrayList allTypes;
      boolean empty;

      public typeData(){
          allTypes = new ArrayList();
      }

      public ArrayList returnData(){
          return allTypes;
      }
      
      public void addTypes(typeParameter type){
          allTypes.add(type);
      }

      public void clearNodes(){
          allTypes.clear();
      }

      public boolean isEmpty(){
          empty=allTypes.isEmpty();
          return empty;
      }

      public void deleteType(int index){
           allTypes.remove(index);
      }


}

