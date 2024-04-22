package com.move.commonData;
/**
   Thesis implementation
   Class : routeParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/

public class routeParameter{

       String id;
       String multi_ref;
       String path;



       public routeParameter(String identification,String multiref,
                          String routepath){

              id           = new String(identification);
              multi_ref    = new String(multiref);
              path         = new String(routepath);

       }

       public String getID(){
           return id;
       }

       public String getMulti_ref(){
           return multi_ref;
       }

       public String getPath(){
           return path;
       }
}

