package com.move.commonData;

import java.util.*;

public class multiCastGroup{

     String id = "";
     Vector memberList = new Vector();

     public multiCastGroup(String groupID){
            id = groupID;
     }

     public void addMember(String member){
            memberList.add(member);
     }

     public String getID(){
         return id;
     }

     public Vector getMember(){
        return memberList;
     }

}

