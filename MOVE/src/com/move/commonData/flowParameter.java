package com.move.commonData;
/**
   Thesis implementation
   Class : flowParameter.java
   Writen By : ZhiHai(Harris) Mo
   Student Number : 3057489
**/


public class flowParameter{

       String id;
       String from;
       String to;
       
       String begin;
       String end;
       String no;
       
       String type;
       

       public flowParameter(String identification, String flowFrom,
                            String flowTo, String flowBegin, String flowEnd,
                            String flowNo){

             id = new String(identification);
             from = new String(flowFrom);
             to = new String(flowTo);
             begin = new String(flowBegin);
             end = new String(flowEnd);
             no = new String(flowNo);

            
       }
       
       public String getID(){
            return id;
       }
       
       public String getFrom(){
           return from;
       }
	   
       public String getTo(){
           return to;
       }
       
       public String getBegin(){
           return begin;
       }
       
       public String getEnd(){
           return end;
       }
       
       public String getNo(){
           return no;
       }
}       
