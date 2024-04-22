package com.move.ns2Part;
/*
 * This program revised by Chien-Ming Chou, NCKU CSIE, LENS lab.
 */
import java.io.*;
import java.util.*;

import com.move.commonData.ReadDumpInfo;
import com.move.commonData.dumpData;
import com.move.commonData.edgeInfo;
import com.move.commonData.laneInfo;
import com.move.commonData.stepInfo;
import com.move.commonData.vehInfo;

public class buildNS {
	private int maxVeh = 10000;
//	boolean debugLog = true;
	boolean debugLog = false;
//	boolean dumpTrace = true;
	boolean dumpTrace = false;
//	boolean dumpRoadEndLocation = true;
	boolean dumpRoadEndLocation = false;

	int node_counter = 0;
	int road_counter = 0;
	List<String> storeNodeList = new ArrayList<String>();
	List<String> storeRoadList = new ArrayList<String>();

	boolean traCImode = false;

	PrintWriter printDebug, printInfo, printRoad;
	int storeArray_counter = 0;
	int storeRoadArray_counter = 0;
	List<String> storeDebugLog = new ArrayList<String>();
	List<String> storeArray = new ArrayList<String>();
	List<String> storeRoadArray = new ArrayList<String>();
	String Debug_file = "Debug_file.txt";
	String Trace_Info_file = "Trace_Info.txt";

	String filename = "";
	String netFile = "";
	String nodefile = "";
	String namfile = "";

	private boolean namflag = false;

	PrintWriter printer;
	private int onlyMovement = 0;
	private String temp = "";
	private String source = "";
	private String sink = "";

	private String chanType;
	private String propType;
	private String netifType;
	private String macType;
	private String queueType;
	private String layerType;
	private String antModel;
	private String packetNo;
	private String protocol;

	private String topoLength;
	private String topoWidth;
	private String agentTriger;
	private String routerTriger;
	private String macTriger;
	private String movementTriger;
	private String stopTime;

	public String temptime = "";
	public String start = "";
	private int hasstart = 0;

	private int currid = 0;

	private int TCPFlag = 0; // This is the flag that indicate whether
	// user sets the TCP/FTP parameters
	private int UDPFlag = 0; // This is the flag that indicate whether
	// user sets the UDP/CBR parameters

	private String udpStart_ = "";
	private String udpStop_ = "";
	// private String udpWindow_= "";
	private String udpPacketSize_ = "";
	private String udpRate_ = "";
	private String udpInterval_ = "";
	private String udpRandom_ = "";
	private String udpMaxpkts_ = "";

	private String tcpStart_ = "";
	private String tcpStop_ = "";
	private String tcpWindow_ = "";
	private String tcpPacketSize_ = "";
	private String tcpMaxburst_ = "";
	private String tcpMaxcwnd_ = "";

	private double connSTime[] = new double[1000];
	private double connETime[] = new double[1000];
	public Vector nodeGroups = new Vector();

	// public Vector SRMGroupID = new Vector();
	// public HashMap SRMGroup = new HashMap();

	Vector initialNode = new Vector();
	Vector nodeMovement = new Vector();
	public Vector connections = new Vector();
	public Vector conneStartTime = new Vector();
	Vector numberedID = new Vector();
	double xBoundary = 0.0;
	double yBoundary = 0.0;

	public buildNS(String traceFile, String netXMLFile, boolean enableTraCI) {
		filename = traceFile;
		netFile = netXMLFile;
		traCImode = enableTraCI;
	}

	public void setOnlyMovement(int target) {
		onlyMovement = target;
	}

	public void addConnections(Vector newConn) {
		connections.addElement(newConn);
	}

	public void connectionTime(int cSTime, int cETime, int index, int size) {
		// connSTime = new int [size+1];
		// System.out.println("_" + cTime);
		connSTime[index] = cSTime;
		connETime[index] = cETime;
	}

	public void addGroupNodes(Vector newGroup) {
		String temp = (String) newGroup.elementAt(2);
		if (temp.length() > 3) {
			nodeGroups.add(temp);
			nodeGroups.add((String) newGroup.elementAt(0));
		}
	}

	/*
	 * public void setSRM(String srmStart, String srmStop, String window, String
	 * packetSize, String burst_time, String idle_time, String rate){ srmStart_ =
	 * srmStart; srmStop_ = srmStop;
	 * 
	 * srmWindow_ = window; //max bound on window size srmPacketSize_ =
	 * packetSize; //constant size of packets generated
	 * 
	 * srmBurst_time_ = burst_time; //average on time for generator
	 * srmIdle_time_ = idle_time; //average off time for generator
	 * 
	 * srmRate_ = rate; //sending rate during on time
	 * 
	 * SRMFlag = 1; }
	 */

	public void setUDP(String udpStart, String udpStop, String window,
			String packetSize, String rate, String interval, String random,
			String maxpkts) {
		udpStart_ = udpStart;
		udpStop_ = udpStop;

		// udpWindow_ = window; // max bound on window size
		udpPacketSize_ = packetSize; // the constant size of the packets
										// generated

		udpRate_ = rate; // sending rate
		udpInterval_ = interval; // interval between packets

		udpRandom_ = random; // whether or not to introduce random noise
		// in the scheduled departure times. defualt is off
		udpMaxpkts_ = maxpkts; // maximum number of packets to send

		UDPFlag = 1;

	}

	public void setTCP(String tcpStart, String tcpStop, String window,
			String packetSize, String maxburst, String maxcwnd) {
		tcpStart_ = tcpStart;
		tcpStop_ = tcpStop;

		tcpWindow_ = window; // max bound on window size
		tcpPacketSize_ = packetSize; // the constant size of the packets
										// generated

		tcpMaxburst_ = maxburst; // max # packets can send back-2-back
		tcpMaxcwnd_ = maxcwnd; // max # cwnd can ever be

		TCPFlag = 1;

	}

	public double returnXBoundary() {
		return xBoundary;
	}

	public double returnYBoundary() {
		return yBoundary;
	}

	public String returnSimTime() {
		return temptime;
	}

	public String returnStartTime() {
		return start;
	}

	public void setNamTrace(boolean flag, String tnamfile) {
		namflag = flag;
		namfile = tnamfile;
	}

	public void setOption(String chan, String prop, String netif, String mac,
			String queue, String layer, String ant, String packet, String pro) {
		chanType = chan;
		propType = prop;
		netifType = netif;
		macType = mac;
		queueType = queue;
		layerType = layer;
		antModel = ant;
		packetNo = packet;
		protocol = pro;
	}

	public void setGodParam(String len, String width, String stop,
			String agent, String route, String mac, String movement) {
		topoLength = len;
		topoWidth = width;
		agentTriger = agent;
		routerTriger = route;
		macTriger = mac;
		movementTriger = movement;
		stopTime = stop;
	}

	public String clearSpace(String l) {
		String result = "";
		int length = l.length();
		int start = 0;

		while (start < length) {
			if ((l.charAt(start) == '<') || (l.charAt(start) == '\n')) {
				break;
			} else
				start++;
		}
		return (l.substring(start));
	}

	public String getValue(String token) {
		int i = token.length();
		int start = 0;
		int end = 0;
		int no = 0;

		for (int j = 0; j < i; j++) {
			if ((token.charAt(j) == '\"') && (no == 0)) {
				no++;
				start = j;
			} else if ((token.charAt(j) == '\"') && (no == 1)) {
				end = j;
				break;
			}
		}
		return (token.substring(start + 1, end));
	}

	public String getValue2(String token) {
		int i = token.length();
		int start = 0;
		int end = 0;
		int no = 0;

		for (int j = 0; j < i; j++) {
			if ((token.charAt(j) == '\"') && (no == 0)) {
				no++;
				start = j;
				break;
			}
		}
		end = token.length();
		return (token.substring(start + 1, end));
	}

	public String getValue3(String token) {
		int i = token.length();
		int start = 0;
		int end = 0;
		int no = 0;

		for (int j = 0; j < i; j++) {
			if ((token.charAt(j) == '\"') && (no == 0)) {
				no++;
				end = j;
				break;
			}
		}
		return (token.substring(start, end));
	}

	public int countToken(String s) {
		int result = 0;
		for (int i = 0; i < s.length(); i++) {
			if (s.charAt(i) == '=')
				result++;
		}
		return (result + 1);
	}

	public String getValueName(String token) {
		int i = token.length();
		int end = 0;
		for (int j = 0; j < i; j++) {
			if (token.charAt(j) == '=') {
				end = j;
				break;
			}
		}
		return (token.substring(0, end));
	}

	public void build() throws Exception {

		long StartTime = System.currentTimeMillis(); // calculate program
														// execution time
		//System.out.println("Processing");
		ReadDumpInfo r = new ReadDumpInfo(filename);
		r.LoadInfo();
		// ReadNode newnodeInfo = new ReadNode(nodefile);
		// newnodeInfo.LoadInfo();
		dumpData d = r.getData();

		ArrayList allSteps = d.returnData();

		String timeID;
		String edgeID;
		String laneID;
		String vehID;
		String speed = " ";
		String RoadMaxSpeed = " ";
		String direction = " ";
		String laneType = "";

		String temx1 = " ", temy1 = " ", temx2 = " ", temy2 = " ";
		String temlength = " ";
		String temposition = " ";
		String fromTemp = "";
		String toTemp = "";
		String tempLaneID = "";

		// This part parse XML.NET file.
		// List<String> variable uses same index in arraryList
		// Here are for edge information
		List<String> listEdge = new ArrayList<String>();
		List<String> listLaneID = new ArrayList<String>();
		List<String> listFrom = new ArrayList<String>();
		List<String> listTo = new ArrayList<String>();
		List<String> listLane_X_from = new ArrayList<String>();
		List<String> listLane_X_to = new ArrayList<String>();
		List<String> listLane_Y_from = new ArrayList<String>();
		List<String> listLane_Y_to = new ArrayList<String>();
		List<String> listLaneMaxSpeed = new ArrayList<String>();
		List<String> listLength = new ArrayList<String>();
		List<String> listLaneType = new ArrayList<String>();

		// Here are for node information
		List<String> listNode = new ArrayList<String>();
		List<String> listX = new ArrayList<String>();
		List<String> listY = new ArrayList<String>();

		// here are for current edge information
		String existVID[] = new String[maxVeh];

		String processEdgeInfo_ID = "";
		String processEdgeInfo_From = "";
		String processEdgeInfo_To = "";

		double offsetX = 15;
		double offsetY = 15;

		for (int initial = 0; initial < maxVeh; initial++) {
			existVID[initial] = "";
		}

		if (debugLog == true) {
			try {
				printDebug = new PrintWriter(new BufferedWriter(new FileWriter(Debug_file), 32768));
			} catch (IOException e) {
				System.out.println("debugLog write error!");
				System.out.println(e);
				System.exit(0);
			}
		}

		if (dumpTrace == true) {
			try {
				printInfo = new PrintWriter(new BufferedWriter(new FileWriter(Trace_Info_file), 32768));
//				printInfo.print("Time Node X_coordinate Y_coordinate speed(m/s) direction current_road\n");
			} catch (IOException e) {
				System.out.println("printInfo dumpTrace write error!");
				System.out.println(e);
				System.exit(0);
			}
		}

		if (dumpRoadEndLocation == true) {
			try {
				printRoad = new PrintWriter(new BufferedWriter(new FileWriter("mapRoadEndLoc.txt"), 32768));
			} catch (IOException e) {
				System.out.println("printRoad write error!");
				System.out.println(e);
				System.exit(0);
			}
		}

		boolean parseFlag = false, catchBoundary = false;

		File roadMap = new File(netFile);
		BufferedReader roadMapContent = new BufferedReader(new FileReader(
				roadMap), 65535);
		StringBuffer tempContent = new StringBuffer();
		String roadMapLine = roadMapContent.readLine();

		while (roadMapLine != null) {
//			 System.out.println(roadMapLine);
			// road-map-line parse

			roadMapLine = clearSpace(roadMapLine);

			if (roadMapLine.equals("<net>"))
				parseFlag = true;

			// System.out.println(roadMapLine.length());
//			 System.out.println(roadMapLine);

			if (roadMapLine.length() >= 7 && parseFlag == true) {
				StringTokenizer stringParse = new StringTokenizer(roadMapLine);
				//System.out.println(stringParse.countTokens());
//				======================================== old version part (sumo 0.10.3)
//				StringTokenizer stringParseType = new StringTokenizer(roadMapLine, "-");
//				//System.out.println(stringParseType.nextToken());
//				if(catchBoundary == false && stringParseType.nextToken().equals("<conv") && stringParse.countTokens() == 1) {
//					// e.g. <conv-boundary>0.00,0.00,1600.00,1600.00</conv-boundary>
//					//System.out.println(roadMapLine);					
//					StringTokenizer stringParseForBoundaryX = new StringTokenizer(roadMapLine, ",");
//					stringParseForBoundaryX.nextToken();
//					stringParseForBoundaryX.nextToken();
//					xBoundary = Double.parseDouble( stringParseForBoundaryX.nextToken() );
//					StringTokenizer stringParseForBoundaryY = new StringTokenizer(stringParseForBoundaryX.nextToken(), "<");
//					yBoundary = Double.parseDouble( stringParseForBoundaryY.nextToken() );
//					catchBoundary = true;
//					//System.out.println(xBoundary);
//					//System.out.println(yBoundary);
//				}
//				======================================== old version part (sumo 0.10.3)

				StringTokenizer stringParseType = new StringTokenizer(roadMapLine);
				//System.out.println(stringParseType.nextToken());
				if (catchBoundary == false && stringParseType.nextToken().equals("<location") && stringParse.countTokens() >= 5) {
					// e.g. <location netOffset="0.00,0.00" convBoundary="0.00,0.00,800.00,800.00" origBoundary="0.00,0.00,800.00,800.00" projParameter="!"/>
//					System.out.println(roadMapLine);					
					StringTokenizer stringParseFindBoundary = new StringTokenizer(roadMapLine, " ");
//					System.out.println("show " + stringParseFindBoundary.nextToken());
					stringParseFindBoundary.nextToken();
					stringParseFindBoundary.nextToken();
					StringTokenizer stringParseForBoundary = new StringTokenizer(stringParseFindBoundary.nextToken(), ",");
					stringParseForBoundary.nextToken();
					stringParseForBoundary.nextToken();
//					System.out.println("show " + stringParseForBoundary.nextToken() );
					xBoundary = Double.parseDouble(stringParseForBoundary.nextToken());
					xBoundary += 50;
					StringTokenizer stringParseForBoundaryY = new StringTokenizer(stringParseForBoundary.nextToken(), "\"");
//					System.out.println("show " + stringParseForBoundaryY.nextToken() );
					yBoundary = Double.parseDouble(stringParseForBoundaryY.nextToken());
					yBoundary += 50;
//					xBoundary = Double.parseDouble( stringParseForBoundaryX.nextToken() );
//					StringTokenizer stringParseForBoundaryY = new StringTokenizer(stringParseForBoundaryX.nextToken(), "<");
//					yBoundary = Double.parseDouble( stringParseForBoundaryY.nextToken() );
					catchBoundary = true;

//					System.out.println(xBoundary);
//					System.out.println(yBoundary);
//					System.in.read();
				}

				if (stringParse.countTokens() > 3) {
					String temp = stringParse.nextToken();
//					 System.out.println(stringParse.countTokens());
//					 System.out.println(temp);

//					System.out.println("processEdgeInfo_ID = " + processEdgeInfo_ID);
//					System.out.println("processEdgeInfo_From = " + processEdgeInfo_From);
//					System.out.println("processEdgeInfo_To = " + processEdgeInfo_To);
//					System.in.read();

					if (temp.compareTo("<edge") == 0 && stringParse.countTokens() == 6) {
//						System.out.println(" - > "+ temp);

//						System.out.println(roadMapLine);

						temp = stringParse.nextToken();
//						System.out.print(" target_1 " + getValue(temp));
						processEdgeInfo_ID = getValue(temp);

						temp = stringParse.nextToken();
//						System.out.print(" target_2 " + getValue(temp));
						processEdgeInfo_From = getValue(temp);

						temp = stringParse.nextToken();
//						System.out.print(" target_3 " + getValue(temp) + "\n");
						processEdgeInfo_To = getValue(temp);

//						System.out.println("processEdgeInfo_ID = " + processEdgeInfo_ID);
//						System.out.println("processEdgeInfo_From = " + processEdgeInfo_From);
//						System.out.println("processEdgeInfo_To = " + processEdgeInfo_To);
//						System.in.read();
					}

					if (temp.compareTo("<lane") == 0 && stringParse.countTokens() >= 6) {
						StringTokenizer stringParse2 = new StringTokenizer(roadMapLine);
						String temp2 = stringParse2.nextToken();
						String tempStr = "";

//						System.out.println("		 processEdgeInfo_ID " + processEdgeInfo_ID);
//						System.out.println(roadMapLine);

						listEdge.add(processEdgeInfo_ID);
						listFrom.add(processEdgeInfo_From);
						listTo.add(processEdgeInfo_To);

						listLaneType.add("Road");

						temp2 = stringParse2.nextToken();
//						System.out.println("1 " + "   " + temp2 + "  "+ getValue(temp2));
						listLaneID.add(getValue(temp2));
//						System.out.println(temp2 + "    listLaneID " + getValue(temp2));
						temp2 = stringParse2.nextToken();

						String checkStr = "";
						int checkStrLen = temp2.length();

						// depart 
						if (checkStrLen > 6) {
							checkStr = temp2.substring(0, 6);
//							System.out.println( " -> " + checkStr);
//							System.out.println(temp2 + "     2 " + getValue(temp2));
							if (checkStr.compareToIgnoreCase("depart") == 0)
								temp2 = stringParse2.nextToken();
						}

						checkStrLen = temp2.length();

						//allow 
						if (checkStrLen > 5) {
							checkStr = temp2.substring(0, 5);
//							System.out.println( " -> " + checkStr);							
							if (checkStr.compareToIgnoreCase("allow") == 0)
								temp2 = stringParse2.nextToken();
						}

//						System.in.read();

//						System.out.println("3 " + getValue(temp2));
						listLaneMaxSpeed.add(getValue(temp2));
//						System.out.println("listLaneMaxSpeed " + getValue(temp2));
						temp2 = stringParse2.nextToken();
//						System.out.println("4 " + getValue(temp2));						
						listLength.add(getValue(temp2));
//						System.out.println("listLength " + getValue(temp2));
						temp2 = stringParse2.nextToken();
//						System.out.println("5 " + temp2);

						tempStr = temp2;
//						System.out.println("from part " + tempStr);

						StringTokenizer stringParse3 = new StringTokenizer(temp2, ",");
						String temp3 = stringParse3.nextToken();

//						System.out.println("a " + getValue2(temp3));

						if (temp3 == null) {
							System.out.println("oops! please inform the author. MOVE parse has bug!");
							System.exit(0);
						} else {
							listLane_X_from.add(getValue2(temp3));
						}

						temp3 = stringParse3.nextToken();
//						System.out.println("b " + temp3);

						if (temp3 == null) {
							System.out.println("oops! please inform the author. MOVE parse has bug!");
							System.exit(0);
						} else {
							listLane_Y_from.add(temp3);
						}

						temp2 = stringParse2.nextToken();
//						System.out.println("to part " + tempStr);

//						System.out.println("    temp2 " + temp2);
						stringParse3 = new StringTokenizer(temp2, ",");
						temp3 = stringParse3.nextToken();
//						System.out.println("c " + temp3);

						if (temp3.length() == 0) {
							System.out.println("oops! please check your map format since MOVE cannot get right data from map file");
							System.exit(0);
						} else {
//							System.out.println(" listLane_X_to = " + temp3);	
							listLane_X_to.add(temp3);
						}

						temp3 = stringParse3.nextToken();
						String inputY = getValue3(temp3);
//						System.out.println(" d  " +   inputY  );

						if (inputY.length() == 0) {
							inputY = temp3;
//							System.out.println("after d  " +   inputY  );
						}

						if (inputY.length() == 0) {
							System.out.println("oops! please check your map format since MOVE cannot get right data from map file");
							System.exit(0);
						} else {
//							System.out.println(" listLane_Y_to = " + inputY);	
							listLane_Y_to.add(inputY);
						}

// 					    System.in.read();
					}

					if (temp.compareTo("<junction") == 0 && stringParse.countTokens() >= 4) {
						temp = stringParse.nextToken();
//						System.out.println(getValue(temp));
						listNode.add(getValue(temp));

						temp = stringParse.nextToken();
//						System.out.println(getValue(temp));
						temp = stringParse.nextToken();
//						System.out.println(getValue(temp));
						listX.add(getValue(temp));

						temp = stringParse.nextToken();
//						System.out.println(getValue(temp));
						listY.add(getValue(temp));
//						System.in.read();
					}
				}
				else if (stringParse.countTokens() == 3) {
					String temp = stringParse.nextToken();
					// System.out.println(stringParse.countTokens());
					// System.out.println(temp);

					if (temp.compareTo("<edge") == 0) {
//						 System.out.println(" - > "+ temp);

						temp = stringParse.nextToken();
//						System.out.println("		 target_1 " + getValue(temp));
//						System.out.println(roadMapLine);
						listEdge.add(getValue(temp));
//						System.in.read();
						listFrom.add(null);
						listTo.add(null);

						listLaneType.add("Intersection");

						// read the edge length
						roadMapLine = roadMapContent.readLine();
						roadMapLine = roadMapContent.readLine();
						roadMapLine = clearSpace(roadMapLine);
//						System.out.println(roadMapLine);
						StringTokenizer stringParse2 = new StringTokenizer(roadMapLine);

						int checkLen = stringParse2.countTokens();
//						System.out.println(checkLen);
						String temp2 = stringParse2.nextToken();
						if (temp2.compareTo("<lane") == 0) {
//							System.out.println(roadMapLine);
							temp2 = stringParse2.nextToken();
//							System.out.println(getValue(temp2));
							listLaneID.add(getValue(temp2));
//							System.out.println(" listLaneID = " + getValue(temp2));
							temp2 = stringParse2.nextToken();
//							 System.out.println(getValue(temp2));
							temp2 = stringParse2.nextToken();
//							 System.out.println(getValue(temp2));
							listLaneMaxSpeed.add(getValue(temp2));
							temp2 = stringParse2.nextToken();
//							 System.out.println("++ " + getValue(temp2));
							listLength.add(getValue(temp2));

							temp2 = stringParse2.nextToken();

							StringTokenizer stringParse3 = new StringTokenizer(temp2, ",");
							String temp3 = stringParse3.nextToken();

//							System.out.println("a " + getValue2(temp3));

							if (temp3 == null) {
								System.out.println("oops! please inform the author. MOVE parse has bug!");
								System.exit(0);
							} else {
								listLane_X_from.add(getValue2(temp3));
							}

							temp3 = stringParse3.nextToken();
//							System.out.println("b " + temp3);

							if (temp3 == null) {
								System.out.println("oops! please inform the author. MOVE parse has bug!");
								System.exit(0);
							} else {
								listLane_Y_from.add(temp3);
							}

							for (int cut = 7; cut < checkLen; cut++) {
								temp2 = stringParse2.nextToken();
							}

							temp2 = stringParse2.nextToken();
//							System.out.println("to part " + temp2);

							stringParse3 = new StringTokenizer(temp2, ",");
							temp3 = stringParse3.nextToken();
//							System.out.println("c " + temp3);

							if (temp3.length() == 0) {
								System.out
										.println("oops! please check your map format since MOVE cannot get right data from map file");
								System.exit(0);
							} else {
								listLane_X_to.add(temp3);
//								System.out.println(" listLane_X_to = " + temp3);
							}

							temp3 = stringParse3.nextToken();
							String inputY = getValue3(temp3);
//							System.out.println("d  " +   inputY  );

							if (inputY.length() == 0) {
								inputY = temp3;
//								System.out.println("after d  " +   inputY  );
							}

							if (inputY.length() == 0) {
								System.out
										.println("oops! please check your map format since MOVE cannot get right data from map file");
								System.exit(0);
							} else {
//								System.out.println(" listLane_Y_to = " + inputY);	
								listLane_Y_to.add(inputY);
							}
						}
//						System.in.read();
					}
				}
			}
			// StringTokenizer rMLParse= new StringTokenizer(roadMapLine);
			// String tt = rMLParse.nextToken();
			roadMapLine = roadMapContent.readLine();
		}

		// test function
		// for(String listOut : listLength) {
		// System.out.print(listOut + " ");
		// }

//		System.in.read();
		System.out.println("xBoundary = " + xBoundary + " yBoundary = " + yBoundary);

		HashMap map = new HashMap();
		int vehicle_counter = 0;

		for (int i = 0; i < allSteps.size(); i++) {
			String Debuglog = "";
			// System.out.println("AllSteps = "+allSteps.size());			

			stepInfo s = (stepInfo) allSteps.get(i);
			ArrayList allEdges = s.returnData();
			timeID = s.getID();
			Debuglog += timeID + " ";

//			 System.out.println("timeID-> " + timeID);

			for (int j = 0; j < allEdges.size(); j++) {
				// System.out.println("AllEdges = "+allEdges.size());
				edgeInfo e = (edgeInfo) allEdges.get(j);
				ArrayList allLanes = e.returnData();
				edgeID = e.getID();
//				System.out.println("edgeID-> " + edgeID);
				Debuglog += edgeID + " ";

				int roadIndex = 10000;
				if (dumpRoadEndLocation == true) {

					int locEdgeIndex = 0;

					for (String findEdge : listEdge) {
						if (listEdge.get(locEdgeIndex).compareTo(edgeID) == 0)
							break;
						locEdgeIndex++;
					}

					double endPosition_X = Double.parseDouble(listLane_X_to.get(locEdgeIndex));
					double endPosition_Y = Double.parseDouble(listLane_Y_to.get(locEdgeIndex));

					endPosition_X += offsetX;
					endPosition_Y += offsetY;

					//************************************* auto add-road function ����������������  ***********************
					boolean findRoad = false;
					if (road_counter == 0) {

						if (listLaneType.get(locEdgeIndex).compareToIgnoreCase("Road") == 0)
							roadIndex = 1000;
						else
							roadIndex = 0;

						storeRoadList.add(edgeID);
						findRoad = true;
						storeRoadArray.add(roadIndex + " " + endPosition_X + " " + endPosition_Y + " " + RoadMaxSpeed);
//						System.out.println("ADD!   " + edgeID + "   " + roadIndex + " " + endPosition_X + " " + endPosition_Y + " " + RoadMaxSpeed);
						road_counter++;
						storeRoadArray_counter++;
					}
					else {
						for (int check = 0; check < road_counter; check++) {
							if (storeRoadList.get(check).compareTo(edgeID) == 0) {
								findRoad = true;
								roadIndex += check;
								break;
							}
						}
					}
					if (findRoad == false) {
						storeRoadList.add(edgeID);

						if (listLaneType.get(locEdgeIndex).compareToIgnoreCase("Road") == 0)
							roadIndex += road_counter;
						else
							roadIndex += (road_counter - 1000);

						storeRoadArray.add(roadIndex + " " + endPosition_X + " " + endPosition_Y + " " + RoadMaxSpeed);
//						System.out.println("ADD!   " + edgeID + "   " + roadIndex + " " + endPosition_X + " " + endPosition_Y + " " + RoadMaxSpeed);
						road_counter++;
						storeRoadArray_counter++;
					}
					//************************************* auto add-road function ������������������������ *************************						
				}

				for (int m = 0; m < allLanes.size(); m++) {
					// System.out.println("AllLanes = "+allLanes.size());
					laneInfo l = (laneInfo) allLanes.get(m);
					ArrayList allVeh = l.returnData();
					laneID = l.getID();
					Debuglog += laneID + " ";
//					System.out.println("laneID-> " + laneID);

					for (int n = 0; n < allVeh.size(); n++) {
						// System.out.println("AllVeh = "+allVeh.size());
						vehInfo v = (vehInfo) allVeh.get(n);
						vehID = v.getID();
						temposition = v.getPosition();
						speed = v.getSpeed();
						Debuglog += vehID + " pos=";
						Debuglog += temposition + " speed=";
						Debuglog += speed + " ";

//						 System.out.println("vehID-> " + vehID);
//						 System.out.println("speed-> " + speed);
//						 System.out.println("temposition-> " + temposition);

						String tempID = "";

						// This part start process XML.NET file to find current
						// location of the vehicle

						int findEdge_index = 0;
						int findNode_index = 0;
						int findLocation_index = 0;
						boolean findNode_flag = false;
						int searchCounter = 0;
						String findTemp;

						temx1 = "";
						temy1 = "";
						temx2 = "";
						temy2 = "";

						// update current vehicle information
						for (int foundnode = 0; vehicle_counter != 0 && foundnode < maxVeh; foundnode++) {
							if (existVID[foundnode].compareTo(vehID) == 0)
							{
								findLocation_index = foundnode;
								findNode_flag = true;
//								System.out.println("!! find exist vehicle ->" + vehID + "  index -> " + findLocation_index);
								break;
							}
						}

						if (findNode_flag == false)
						{
							existVID[vehicle_counter] = vehID;
//							System.out.println("cannot find exist vehicle ->" + vehID + "  index -> " + vehicle_counter);
						}

						boolean getData = false;
						for (String findEdge : listEdge) {
							findTemp = listEdge.get(findEdge_index);
							fromTemp = listFrom.get(findEdge_index);
							toTemp = listTo.get(findEdge_index);
							temlength = listLength.get(findEdge_index);
							tempLaneID = listLaneID.get(findEdge_index);
							RoadMaxSpeed = listLaneMaxSpeed.get(findEdge_index);
//							System.out.println("current check EdgeID = " + findTemp + "  tempLaneID = " + tempLaneID + "  edgeID = " + edgeID);
							if (findTemp.compareTo(edgeID) == 0) {
//								System.out.println("		current check EdgeID = " + findTemp + "  tempLaneID = " + tempLaneID + "  edgeID = " + edgeID);
								// the vehicle is moving on intersection
								if (tempLaneID.compareTo(laneID) == 0) {
//									 System.out.println("current check EdgeID = " + findTemp + "  tempLaneID = " + tempLaneID);
									if (findNode_flag == true) {
//										 System.out.println(" --> index (a) " + findLocation_index);
										getData = true;
										temx1 = listLane_X_from.get(findEdge_index);
										temx2 = listLane_X_to.get(findEdge_index);
										temy1 = listLane_Y_from.get(findEdge_index);
										temy2 = listLane_Y_to.get(findEdge_index);
										laneType = listLaneType.get(findEdge_index);

										Debuglog += findTemp + " ";
										Debuglog += tempLaneID + " from=";
										Debuglog += fromTemp + " to=";
										Debuglog += toTemp + " length=";
										Debuglog += temlength + " x1=";
										Debuglog += temx1 + " x2=";
										Debuglog += temx2 + " y1=";
										Debuglog += temy1 + " y2=";
										Debuglog += temy2 + " mode=";

										break;
									} else {
//										 System.out.println(" --> index (b) " + vehicle_counter);
										getData = true;
										temx1 = listLane_X_from.get(findEdge_index);
										temx2 = listLane_X_to.get(findEdge_index);
										temy1 = listLane_Y_from.get(findEdge_index);
										temy2 = listLane_Y_to.get(findEdge_index);
										laneType = listLaneType.get(findEdge_index);
										vehicle_counter++;

//										 System.out.print("edgeID -> " + findTemp + "  LaneID = " + tempLaneID + " ");
//										 System.out.print(fromTemp + " " + toTemp + " ");								 
//										 System.out.println(" L-> " + temlength );
//										 System.out.println(" x from -> " + listLane_X_from.get(findEdge_index));
//										 System.out.println(" x to -> " + listLane_X_to.get(findEdge_index));
//										 System.out.println(" y from -> " + listLane_Y_from.get(findEdge_index));
//										 System.out.println(" y to -> " + listLane_Y_to.get(findEdge_index) + "\n");

										Debuglog += findTemp + " ";
										Debuglog += tempLaneID + " from=";
										Debuglog += fromTemp + " to=";
										Debuglog += toTemp + " length=";
										Debuglog += temlength + " x1=";
										Debuglog += temx1 + " x2=";
										Debuglog += temx2 + " y1=";
										Debuglog += temy1 + " y2=";
										Debuglog += temy2 + " mode=";

										break;
									}
								}

								// if fromTemp is null, the data is handle for intersection node
//								 if(fromTemp == null)
//								 	System.in.read();

//								 System.out.print("edgeID -> " + findTemp + "  LaneID = " + tempLaneID + " ");
//								 System.out.print(fromTemp + " " + toTemp + " ");								 
//								 System.out.println(" L-> " + temlength );
//								 System.out.println(" x from -> " + listLane_X_from.get(findEdge_index));
//								 System.out.println(" x to -> " + listLane_X_to.get(findEdge_index));
//								 System.out.println(" y from -> " + listLane_Y_from.get(findEdge_index));
//								 System.out.println(" y to -> " + listLane_Y_to.get(findEdge_index) + "\n");

							}
							findEdge_index++;
//							System.in.read();
						}

						if (getData == false) {
							System.out.println("map information capture fail.");
							System.exit(0);
						}

//						System.in.read();

						double douTimeVal = Double.parseDouble(timeID);
						int intTimeVal = (int) douTimeVal;
						Double xfrom, yfrom, xto, yto, position, length;
						Double extra_pos, extra_len;
						double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
						double ratio = 0.0;

						double a = 0.0, b = 0.0;

						xfrom = new Double(temx1);
						xto = new Double(temx2);

						yfrom = new Double(temy1);
						yto = new Double(temy2);

//						if( xfrom < 0) {
//							xfrom = Math.abs(xfrom);
//						}
//						if( xto < 0) {
//							xto = Math.abs(xto);
//						}
//						if( yfrom < 0) {
//							yfrom = Math.abs(yfrom); 
//						}
//						if( yto < 0) {
//							yto = Math.abs(yto); 
//						}

						xfrom += offsetX;
						xto += offsetX;
						yfrom += offsetY;
						yto += offsetY;

//						if (xto < 0 || yto < 0 || xfrom < 0 || yfrom < 0) {
//						    System.out.println("x from = " + xfrom + " x to = " + xto + " y from = " + yfrom + " y to = " + yto);
//						}

						position = new Double(temposition);
						length = new Double(temlength);

//						System.out.println("pos =" + position + " length =" + length);
//						System.in.read();

						if ((xfrom.doubleValue() < xto.doubleValue())
								&& (yfrom.doubleValue() != yto.doubleValue())) {
							// System.out.println("1");
							x1 = xfrom.doubleValue();
							y1 = yfrom.doubleValue();

							x2 = xto.doubleValue();
							y2 = yto.doubleValue();

							ratio = position.doubleValue() / length.doubleValue();

							a = x1 + ratio * (x2 - x1);

							if (y2 > y1) {
								b = y1 + ratio * (y2 - y1);
								direction = "southwest-to-northeast";
							}
							else {
								b = y1 - ratio * (y1 - y2);
								direction = "northwest-to-southeast";
							}
							Debuglog += "1" + " a=";

						} else if ((xfrom.doubleValue() > xto.doubleValue())
								&& (yfrom.doubleValue() != yto.doubleValue())) {
							// System.out.println("2 pos = " + position + " x
							// from = " + xfrom + " x to = " + xto + " y from =
							// " + yfrom + " y to = " + yto);
							x1 = xto.doubleValue();
							y1 = yto.doubleValue();

							x2 = xfrom.doubleValue();
							y2 = yfrom.doubleValue();

							ratio = position.doubleValue() / length.doubleValue();

							a = x2 - ratio * (x2 - x1);

							if (y2 > y1) {
								b = y2 - ratio * (y2 - y1);
								direction = "northeast-to-southwest";
							}
							else {
								b = y2 + ratio * (y1 - y2);
								direction = "southeast-to-northwest";
							}

							Debuglog += "2" + " ratio=";
							Debuglog += ratio + " position=";
							Debuglog += position + " length=";
							Debuglog += length + " a=";

						}

						if ((xfrom.doubleValue() == xto.doubleValue())
								&& (yfrom.doubleValue() < yto.doubleValue())) {
							// System.out.println("3");
							direction = "North";
							x1 = xfrom.doubleValue();
							y1 = yfrom.doubleValue();

							x2 = xto.doubleValue();
							y2 = yto.doubleValue();

							a = x1;
							b = y1 + position.doubleValue();
							Debuglog += "3" + " a=";
						} else if ((xfrom.doubleValue() == xto.doubleValue())
								&& (yfrom.doubleValue() > yto.doubleValue())) {
							// System.out.println("4");
							direction = "South";
							x1 = xto.doubleValue();
							y1 = yto.doubleValue();

							x2 = xfrom.doubleValue();
							y2 = yfrom.doubleValue();

							a = x1;
							b = y2 - position.doubleValue();
							Debuglog += "4" + " a=";
							// b =
							// y1+length.doubleValue()-position.doubleValue();
						} else if ((yfrom.doubleValue() == yto.doubleValue())
								&& (xfrom.doubleValue() < xto.doubleValue())) {
							// System.out.println("5");
							direction = "East";
							x1 = xfrom.doubleValue();
							y1 = yfrom.doubleValue();

							x2 = xto.doubleValue();
							y2 = yto.doubleValue();

							a = x1 + position.doubleValue();
							b = y1;
							Debuglog += "5" + " a=";
						} else if ((yfrom.doubleValue() == yto.doubleValue())
								&& (xfrom.doubleValue() > xto.doubleValue())) {
							// System.out.println("6");
							direction = "West";
							x1 = xto.doubleValue();
							y1 = yto.doubleValue();

							x2 = xfrom.doubleValue();
							y2 = yfrom.doubleValue();

							// a =
							// x1+length.doubleValue()-position.doubleValue();
							a = x2 - position.doubleValue();
							b = y1;
							Debuglog += "6" + " a=";
						}

						if (map.isEmpty()) {
//							System.out.println(a+ " " + b);
							NSNode tempNode = new NSNode(timeID, vehID, a, b);
							map.put(vehID, tempNode);
							initialNode.add(tempNode);
//							System.out.println(timeID + " " + vehID + " " + a + " " + b );

							Vector idnode = new Vector();
							idnode.addElement(new Integer(currid));
							idnode.addElement(vehID);
							idnode.addElement(timeID);
							numberedID.addElement(idnode);
							currid++;
							temptime = timeID;

							if (hasstart == 0) {
								start = timeID;
								hasstart = 1;
							}

						} else {
							if (map.containsKey(vehID)) {
								// System.out.println(a+ " " + b);
								NSNode tempNode = new NSNode(timeID, vehID, a,
										b);
								tempNode.setSpeed(speed);
								tempNode.setTimestep(timeID);
								nodeMovement.add(tempNode);
								temptime = timeID;
								// System.out.println(timeID);
							} else {
								NSNode tempNode = new NSNode(timeID, vehID, a,
										b);
								map.put(vehID, tempNode);
								initialNode.add(tempNode);

								Vector idnode = new Vector();
								idnode.addElement(new Integer(currid));
								idnode.addElement(vehID);
								numberedID.addElement(idnode);
								currid++;
								temptime = timeID;
								// System.out.println(timeID);
								if (hasstart == 0) {
									start = timeID;
									hasstart = 1;
								}
							}
						}

						if (dumpTrace == true) {
							String tempStr = "";
							if (dumpRoadEndLocation == true) {
								//************************************* auto add-node function ����������������  ***********************
								boolean findNode = false;
								int nodeIndex = 0;
								if (node_counter == 0) {
									nodeIndex = 0;
									storeNodeList.add(vehID);
									findNode = true;
									node_counter++;
								}
								else {
									for (int check = 0; check < node_counter; check++) {
										if (storeNodeList.get(check).compareTo(vehID) == 0) {
											findNode = true;
											nodeIndex = check;
											break;
										}
									}
								}
								if (findNode == false) {
									storeNodeList.add(vehID);
									nodeIndex = node_counter;
									node_counter++;
								}
								//************************************* auto add-node function ������������������������ *************************

								//1:east, 2:west, 3:north, 4:south
								int numDirection = 0;
								if (direction.compareToIgnoreCase("East") == 0)
									numDirection = 1;
								else if (direction.compareToIgnoreCase("West") == 0)
									numDirection = 2;
								else if (direction.compareToIgnoreCase("North") == 0)
									numDirection = 3;
								else if (direction.compareToIgnoreCase("South") == 0)
									numDirection = 4;
								else if (direction.compareToIgnoreCase("southwest-to-northeast") == 0)
									numDirection = 1;
								else if (direction.compareToIgnoreCase("northeast-to-southwest") == 0)
									numDirection = 2;
								else if (direction.compareToIgnoreCase("southeast-to-northwest") == 0)
									numDirection = 1;
								else if (direction.compareToIgnoreCase("northwest-to-southeast") == 0)
									numDirection = 2;

//								tempStr = String.valueOf(intTimeVal) + " " + nodeIndex + " " + vehID + " " + a + " " + b + " " + speed + " " + numDirection + " " + roadIndex + " " + edgeID;
								tempStr = String.valueOf(intTimeVal) + " " + nodeIndex + " " + a + " " + b + " " + speed + " "
										+ numDirection + " " + roadIndex;

							} else {
								tempStr = String.valueOf(intTimeVal) + " " + vehID + " " + a + " " + b + " " + speed + " " + direction
										+ " " + edgeID;
							}

							storeArray.add(tempStr);
							storeArray_counter++;
//							printInfo.print(timeID + " " + vehID + " " + a + " " + b + " " + speed + " " + direction + " " + edgeID + "\n");

//							System.out.println(timeID + " " + vehID + " " + a + " " + b + " " + speed + " " + direction + " " + edgeID + "\n");
//							System.in.read();
						}

						Debuglog += a + " b=";
						Debuglog += b + " ";
						if (debugLog == true) {
							storeDebugLog.add(Debuglog);
						}
						Debuglog = " ";
					}
				}
			}
		}
		// System.out.println("over");
		long ProcessTime = System.currentTimeMillis() - StartTime;
		System.out.println("Translation time (ms) = " + ProcessTime);
		System.out.println("offsetX = " + offsetX + " offsetY = " + offsetY);
		d = null;
		r = null;
		allSteps = null;
		existVID = null;

//		try {
//			printInfo = new PrintWriter(new BufferedWriter(new FileWriter(Trace_Info_file), 32768));
//			printInfo.print("Time Node X_coordinate Y_coordinate speed(m/s) direction current_road\n");
//		} catch (IOException e) {
//			System.out.println("printInfo write error!");
//		}

		if (debugLog == true) {
			int temp_counter = 0;
			for (String findstoreInforimation : storeDebugLog) {
				printDebug.println(storeDebugLog.get(temp_counter));
				temp_counter++;
			}
			printDebug.close();
		}

		if (dumpTrace == true) {
			int temp_counter = 0;
			for (String findstoreInforimation : storeArray) {
				printInfo.println(storeArray.get(temp_counter));
				temp_counter++;
			}
			printInfo.close();
		}

		if (dumpRoadEndLocation == true) {
			int temp_counter = 0;
			for (String findstoreInforimation : storeRoadArray) {
				printRoad.println(storeRoadArray.get(temp_counter));
				temp_counter++;
			}
			printRoad.close();
		}

	}

	public void write(String tclFile, String traceFile) {

		// For creat node move pattern
		int NodeCheckFlag[] = new int[numberedID.size() + 1];
		int NodeCountFlag[] = new int[numberedID.size() + 1];

		// System.out.println(numberedID.size());

		try {
			printer = new PrintWriter(new BufferedWriter(
					new FileWriter(tclFile), 32768));
		} catch (IOException e) {
			System.out.println("print error!");
		}

		float localNodeTime;
		if (traCImode == true) {
			printer.print("# ======================================================================\n");
			printer.print("# Define options\n");
			printer.print("# ======================================================================\n");

			printer.print("set val(chan)   " + chanType
					+ "		      ;# channel type\n");
			printer.print("set val(prop)   " + propType
					+ "		      ;# radio-propagation model\n");
			printer.print("set val(netif)  " + netifType
					+ "		      ;# network interface type\n");
			printer.print("set val(mac)    " + macType
					+ "		      ;# MAC type\n");
			printer.print("set val(ifq)    " + queueType
					+ "		      ;# interface queue type\n");
			printer.print("set val(ll)     " + layerType
					+ "		      ;# link layer type\n");
			printer.print("set val(ant)    " + antModel
					+ "		      ;# antenna model\n");
			printer.print("set val(ifqlen) " + packetNo
					+ "		      ;# max packet in ifq\n");
			printer.print("set val(nn)     " + initialNode.size()
					+ "	      ;# number of mobilenodes\n");
			printer.print("set val(rp)     " + protocol
					+ "		      ;# routing protocol\n\n");

			printer.print("set opt(x)      " + topoLength
					+ "			      ;# x coordinate of topology\n");
			printer.print("set opt(y)      " + topoWidth
					+ "			     ;# y coordinate of topology\n");
			printer.println("set stopTime      " + stopTime);
			printer.println("");

			printer.print("# ======================================================================\n");
			printer.print("# Main Program\n");
			printer.print("# ======================================================================\n");

			printer.print("# \n");
			printer.print("# Initialize Global Variables\n");
			printer.print("# \n");

			printer.print("set ns_ [new Simulator]\n");
			printer.print("set tracefd [open " + traceFile + " w]\n");
			printer.print("$ns_ trace-all $tracefd\n\n");

			if (namflag == true) {
				printer.print("set namtrace [open " + namfile + " w]\n");
				printer
						.print("$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)\n\n");
			}

			printer.print("# set up topography object\n");
			printer.print("set topo    [new Topography]\n");
			printer.print("$topo load_flatgrid $opt(x) $opt(y)\n\n");

			printer.print("# \n");
			printer.print("# Create God\n");
			printer.print("# \n");

			printer.print("create-god $val(nn)\n\n");

			printer.println("# ======================================================================");
			printer.println("#       TraCI Connection Setup");
			printer.println("# ======================================================================");
			printer.println("set mobilityInterfaceClient [new TraCIClient]");
			printer.println("$mobilityInterfaceClient set-remoteHost localhost");
			printer.println("$mobilityInterfaceClient set-remotePort 8888");
			printer.println("$mobilityInterfaceClient set-timeInterval 1.0");
			printer.println("puts \"Connect to TraCI server\"");
			printer.println("$mobilityInterfaceClient connect");
			printer.println("$mobilityInterfaceClient startSimStepHandler");
			printer.println("");

			printer.print("# Configure node\n\n");

			printer.print("set chan_1_ [new $val(chan)]\n");
			printer.print("$ns_ node-config  -adhocRouting $val(rp) \\\n");
			printer.print(" 		 -llType $val(ll) \\\n");
			printer.print(" 		 -macType $val(mac) \\\n");
			printer.print(" 		 -ifqType $val(ifq) \\\n");
			printer.print(" 		 -ifqLen $val(ifqlen) \\\n");
			printer.print(" 		 -antType $val(ant) \\\n");
			printer.print(" 		 -propType $val(prop) \\\n");
			printer.print(" 		 -phyType $val(netif) \\\n");
			printer.print(" 		 -topoInstance $topo \\\n");
			printer.print(" 		 -agentTrace " + agentTriger + " \\\n");
			printer.print(" 		 -routerTrace " + routerTriger + " \\\n");
			printer.print(" 		 -macTrace " + macTriger + " \\\n");
			printer.print(" 		 -movementTrace " + movementTriger + " \\\n");
			printer.print(" 		 -channel $chan_1_  \n\n");

			printer.print("for {set i 0} {$i < $val(nn)} {incr i} {\n");
			printer.print("  set node_($i) [$ns_ node]\n");
			printer.print("  $node_($i) random-motion 0 ;# disable random motion\n");
			printer.println("  $mobilityInterfaceClient add-node $node_($i)");
			printer.print("}\n");
			printer.print("# \n");
			printer
					.print("# Provide initial (X,Y, for now Z=0) co-ordinates for mobilenodes\n");
			printer.print("# \n");

			// initial flag
			for (int ind = 0; ind < numberedID.size(); ind++) {
				NodeCheckFlag[ind] = 0;
				NodeCountFlag[ind] = -1;
			}

			int listLength = initialNode.size();
			printer.print("# predefine node in NAM  \n");

			double LocX = 0;
			double LocY = 0;

			for (int index = 0; index < listLength; index++) {
				NSNode temNode = (NSNode) initialNode.get(index);
				temp = "";
				for (int ind = 0; ind < numberedID.size(); ind++) {
					if (temNode.getID().equalsIgnoreCase(
							(String) ((Vector) (numberedID.elementAt(ind)))
									.elementAt(1))) {
						temp = temp
								+ ((Vector) (numberedID.elementAt(ind)))
										.elementAt(0);
						break;
					}
				}

				LocX = temNode.getX();
				LocY = temNode.getY();

				if (LocX == 0.0) {
					LocX = 0.0005;
				}
				if (LocY == 0.0) {
					LocY = 0.0005;
				}

				// System.out.println(Integer.parseInt(topoWidth));
				printer.println("  # ID of SUMO: " + temNode.getID());
				printer.print("$node_(" + temp + ") set X_ " + LocX + "\n");
				printer.print("$node_(" + temp + ") set Y_ " + LocY + "\n");
				printer.print("$node_(" + temp + ") set Z_ 0.0\n");

				printer.print("$node_(" + temp + ") setdest " + LocX + " "
						+ LocY + " 1");
				printer.print("\n");
			}

			printer.println("\n# All nodes enable command-enablePositionUpdate\n");

			printer.println("for {set i 1} {$i < $stopTime } {incr i} {");
			for (int index = 0; index < listLength; index++) {
				NSNode temNode = (NSNode) initialNode.get(index);
				printer.println("  $ns_ at $i \"$mobilityInterfaceClient command-enablePositionUpdate " + index + " "
						+ temNode.getID() + "\"");
			}
			printer.println("}\n");

			printer.println("");
			printer.println("# The example of TraCI API");
			printer.println("# Please use MOVE's traCI example scenario to test TraCI's running environment");
			printer.println("");
			printer.println("# command-GetRoadID [nsID] [sumoID]");
			printer.println("# Returns the id of the edge the named vehicle was at within the last step; error value: \"\"");
			printer.println("# $ns_ at 3.0 \"$mobilityInterfaceClient command-GetRoadID 6 flow20A_0\"");
			printer.println("");
			printer.println("# command-SetMaxSpeed [nsID] [sumoID] [max speed]");
			printer.println("# Sets the vehicle's maximum speed to the given value");
			printer.println("# $ns_ at 5.0 \"$mobilityInterfaceClient command-SetMaxSpeed 0 flow60A_0 5\"");
			printer.println("");
			printer.println("# command-ChangeTarget [nsID] [sumoID] [target edge]");
			printer.println("# The vehicle's destination edge is set to the given. The route is rebuilt.");
			printer.println("# $ns_ at 5.0 \"$mobilityInterfaceClient command-ChangeTarget 0 flow60A_0 4344-2\"");
			printer.println("");
			printer.println("# command-changeRoute [nsID] [sumoID] [number of edges] [edges lists] ");
			printer.println("# Assigns the list of edges as the vehicle's new route assuming the first edge given " +
					"is the one the vehicle is curently at: The first occurence of the edge is currently at is searched" +
					" within the new route; the vehicle continues the route from this point in the route from. " +
					"If the edge the vehicle is currently does not exist within the new route, an error is generated.");
			printer.println("# $ns_ at 5.0 \"$mobilityInterfaceClient command-changeRoute 0 flow60A_0 7 0111-1,1011-2,1020-1,2021-1,2131-1,3141-1,3141-2\"");
			printer.println("");
			/* Only the movements are required */
			if (onlyMovement == 1) {
				printer.close();
				return;
			}

			// System.out.println(connections.size());
			if (connections.size() > 0) {
				printer.print("\n# Setup traffic flow between nodes\n");
				Vector vtemp;
				int agentNO = 0;
				int applicationNO = 0;
				int firstSCTP = 0;

				if (TCPFlag == 1) {
					printer.print("Agent/TCP set window_ " + tcpWindow_ + "\n");
					printer.print("Agent/TCP set packetSize_ " + tcpPacketSize_
							+ "\n");
					printer.print("Agent/TCP set maxburst_ " + tcpMaxburst_
							+ "\n");
					printer.print("Agent/TCP set maxcwnd_ " + tcpMaxcwnd_
							+ "\n");
				}
				// UDPFlag = 0;
				if (UDPFlag == 1) {
					// printer.print("Agent/UDP set window_ "+udpWindow_+"\n");
					printer.print("Agent/UDP set packetSize_ " + udpPacketSize_
							+ "\n");
					printer.print("Application/Traffic/CBR set rate_ "
							+ udpRate_ + "\n");
					// printer.print("Application/Traffic/CBR set interval_
					// "+udpInterval_+"\n");
					printer.print("Application/Traffic/CBR set random_ "
							+ udpRandom_ + "\n");
					printer.print("Application/Traffic/CBR set maxpkts_ "
							+ udpMaxpkts_ + "\n");
				}

				//		System.out.println( " connections.size() - " + connections.size());
				for (int i = 0; i < connections.size(); i++) {
					// System.out.println(connSTime[i]);

					vtemp = (Vector) connections.elementAt(i);
					String temp2 = (String) vtemp.elementAt(0);
					String temp3 = (String) vtemp.elementAt(2);

					source = "";
					for (int ind = 0; ind < numberedID.size(); ind++) {
						if (temp2
								.equalsIgnoreCase((String) ((Vector) (numberedID
										.elementAt(ind))).elementAt(1))) {
							source = source
									+ ((Vector) (numberedID.elementAt(ind)))
											.elementAt(0);
							break;
						}
					}

					sink = "";
					for (int ind = 0; ind < numberedID.size(); ind++) {
						if (temp3
								.equalsIgnoreCase((String) ((Vector) (numberedID
										.elementAt(ind))).elementAt(1))) {
							sink = sink
									+ ((Vector) (numberedID.elementAt(ind)))
											.elementAt(0);
							break;
						}
					}

					String agentName = (String) vtemp.elementAt(4);
					if (agentName.compareTo("tcp") == 0) {

						printer.print("\nset " + "tcp" + agentNO
								+ " [new Agent/TCP]\n");
						printer.print("$tcp" + agentNO + " set class_ 2\n");
						printer.print("set sink" + agentNO
								+ " [new Agent/TCPSink]\n");
						printer.print("$ns_ attach-agent $node_(" + source
								+ ") $tcp" + agentNO + "\n");
						printer.print("$ns_ attach-agent $node_(" + sink
								+ ") $sink" + agentNO + "\n");
						printer.print("$ns_ connect $tcp" + agentNO + " $sink"
								+ agentNO + "\n");
						printer.print("set ftp" + applicationNO
								+ " [new Application/FTP]\n");
						printer.print("$ftp" + applicationNO
								+ " attach-agent $tcp" + agentNO + "\n");
						if (TCPFlag == 1) {
							printer.print("$ns_ at " + connSTime[i] + " \"$ftp"
									+ applicationNO + " start\"\n\n");
							printer.print("$ns_ at " + connETime[i] + " \"$ftp"
									+ applicationNO + " stop\"\n\n");
						} else
							printer.print("$ns_ at " + tcpStart_ + " \"$ftp"
									+ applicationNO + " start\"\n\n");

					} else if (agentName.compareTo("udp") == 0) {
						printer.print("\nset udp" + agentNO
								+ " [new Agent/UDP]\n");
						printer.print("$ns_ attach-agent $node_(" + source
								+ ") $udp" + agentNO + "\n");
						printer.print("set cbr" + applicationNO
								+ " [new Application/Traffic/CBR]\n");
						printer.print("$cbr" + applicationNO
								+ " attach-agent $udp" + agentNO + "\n");
						printer.print("set null" + agentNO
								+ " [new Agent/Null]\n");
						printer.print("$ns_ attach-agent $node_(" + sink
								+ ") $null" + agentNO + "\n");
						printer.print("$ns_ connect $udp" + agentNO + " $null"
								+ agentNO + "\n");
						if (UDPFlag == 1) {
							if (Double.parseDouble(udpStart_) < connSTime[i]) {
								printer.print("$ns_ at " + connSTime[i]
										+ " \"$cbr" + applicationNO
										+ " start\"\n\n");
							} else
								printer.print("$ns_ at " + udpStart_
										+ " \"$cbr" + applicationNO
										+ " start\"\n\n");
							printer.print("$ns_ at " + connETime[i] + " \"$cbr"
									+ applicationNO + " stop\"\n\n");
						} else
							printer.print("$ns_ at " + udpStart_ + " \"$cbr"
									+ applicationNO + " start\"\n\n");

					}
					agentNO++;
					applicationNO++;
				}
			}

			printer.print("#\n");
			printer.print("# Tell nodes when the simulation ends\n");
			printer.print("#\n");
			printer.print("for {set i 0} {$i < $val(nn) } {incr i} {\n");
			printer.print("    $ns_ at $stopTime \"$node_($i) reset\";\n");
			printer.print("}\n");
			printer.println("$ns_ at $stopTime \"$mobilityInterfaceClient close\"");
			printer.print("$ns_ at $stopTime \"stop\"\n");
			printer.print("$ns_ at $stopTime \"puts \\\"NS EXITING...\\\" ; $ns_ halt\"\n");
			printer.print("proc stop {} {\n");
			printer.print("    global ns_ tracefd");
			if (namflag == true)
				printer.print(" namtrace\n");
			else
				printer.print("\n");
			printer.print("    $ns_ flush-trace\n");
			printer.print("    close $tracefd\n");
			if (namflag == true)
				printer.print("    close $namtrace\n");
			printer.print("}\n\n");
			printer.print("puts \"Starting Simulation...\"\n");
			printer.print("$ns_ run\n\n");
			printer.close();

			connections.clear();
			conneStartTime.clear();
			return;
		} else {
			// no traCI mode part
			printer
					.print("# ======================================================================\n");
			printer.print("# Define options\n");
			printer
					.print("# ======================================================================\n");

			printer.print("set val(chan)   " + chanType
					+ "		      ;# channel type\n");
			printer.print("set val(prop)   " + propType
					+ "		      ;# radio-propagation model\n");
			printer.print("set val(netif)  " + netifType
					+ "		      ;# network interface type\n");
			printer.print("set val(mac)    " + macType
					+ "		      ;# MAC type\n");
			printer.print("set val(ifq)    " + queueType
					+ "		      ;# interface queue type\n");
			printer.print("set val(ll)     " + layerType
					+ "		      ;# link layer type\n");
			printer.print("set val(ant)    " + antModel
					+ "		      ;# antenna model\n");
			printer.print("set val(ifqlen) " + packetNo
					+ "		      ;# max packet in ifq\n");
			printer.print("set val(nn)     " + initialNode.size()
					+ "	      ;# number of mobilenodes\n");
			printer.print("set val(rp)     " + protocol
					+ "		      ;# routing protocol\n\n");

			printer.print("set opt(x)      " + topoLength
					+ "			      ;# x coordinate of topology\n");
			printer.print("set opt(y)      " + topoWidth
					+ "			     ;# y coordinate of topology\n");

			printer.println("set stopTime      " + stopTime);
			printer.println("");

			printer
					.print("# ======================================================================\n");
			printer.print("# Main Program\n");
			printer
					.print("# ======================================================================\n");

			printer.print("# \n");
			printer.print("# Initialize Global Variables\n");
			printer.print("# \n");

			printer.print("set ns_ [new Simulator]\n");
			printer.print("set tracefd [open " + traceFile + " w]\n");
			printer.print("$ns_ trace-all $tracefd\n\n");

			if (namflag == true) {
				printer.print("set namtrace [open " + namfile + " w]\n");
				printer
						.print("$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)\n\n");
			}

			printer.print("# set up topography object\n");
			printer.print("set topo    [new Topography]\n");
			printer.print("$topo load_flatgrid $opt(x) $opt(y)\n\n");

			printer.print("# \n");
			printer.print("# Create God\n");
			printer.print("# \n");

			printer.print("create-god $val(nn)\n\n");

			printer.print("# Configure node\n\n");

			printer.print("set chan_1_ [new $val(chan)]\n");
			printer.print("$ns_ node-config  -adhocRouting $val(rp) \\\n");
			printer.print(" 		 -llType $val(ll) \\\n");
			printer.print(" 		 -macType $val(mac) \\\n");
			printer.print(" 		 -ifqType $val(ifq) \\\n");
			printer.print(" 		 -ifqLen $val(ifqlen) \\\n");
			printer.print(" 		 -antType $val(ant) \\\n");
			printer.print(" 		 -propType $val(prop) \\\n");
			printer.print(" 		 -phyType $val(netif) \\\n");
			printer.print(" 		 -topoInstance $topo \\\n");
			printer.print(" 		 -agentTrace " + agentTriger + " \\\n");
			printer.print(" 		 -routerTrace " + routerTriger + " \\\n");
			printer.print(" 		 -macTrace " + macTriger + " \\\n");
			printer.print(" 		 -movementTrace " + movementTriger + " \\\n");
			printer.print(" 		 -channel $chan_1_  \n\n");

			printer.print("for {set i 0} {$i < $val(nn)} {incr i} {\n");
			printer.print("  set node_($i) [$ns_ node]\n");
			printer
					.print("  $node_($i) random-motion 0 ;# disable random motion\n");
			printer.print("}\n");
			printer.print("# \n");
			printer
					.print("# Provide initial (X,Y, for now Z=0) co-ordinates for mobilenodes\n");
			printer.print("# \n");

			// initial flag
			for (int ind = 0; ind < numberedID.size(); ind++) {
				NodeCheckFlag[ind] = 0;
				NodeCountFlag[ind] = -1;
			}

			int listLength = initialNode.size();
			// old function
			// /*
			printer.print("#predefine node in NAM  \n");

			double LocX = 0;
			double LocY = 0;

			for (int index = 0; index < listLength; index++) {
				NSNode temNode = (NSNode) initialNode.get(index);
				temp = "";
				for (int ind = 0; ind < numberedID.size(); ind++) {
					if (temNode.getID().equalsIgnoreCase(
							(String) ((Vector) (numberedID.elementAt(ind)))
									.elementAt(1))) {
						temp = temp
								+ ((Vector) (numberedID.elementAt(ind)))
										.elementAt(0);
						break;
					}
				}

				LocX = temNode.getX();
				LocY = temNode.getY();

				if (LocX == 0.0) {
					LocX = 0.0005;
				}
				if (LocY == 0.0) {
					LocY = 0.0005;
				}

				// System.out.println(Integer.parseInt(topoWidth));
				printer.println("  # ID of SUMO: " + temNode.getID());
				printer.print("$node_(" + temp + ") set X_ " + LocX + "\n");
				printer.print("$node_(" + temp + ") set Y_ " + LocY + "\n");
				printer.print("$node_(" + temp + ") set Z_ 0.0\n");

				printer.print("$node_(" + temp + ") setdest " + LocX + " "
						+ LocY + " 1");
				printer.print("\n");
			}
			// */
			// old function

			printer.print("# Now produce node movements\n");

			listLength = nodeMovement.size();
			// System.out.println(listLength);

			for (int index = 0; index < listLength; index++) {
				NSNode temNode = (NSNode) nodeMovement.get(index);
				temp = "";
				for (int ind = 0; ind < numberedID.size(); ind++) {
					if (temNode.getID().equalsIgnoreCase(
							(String) ((Vector) (numberedID.elementAt(ind)))
									.elementAt(1))) {
						temp = temp
								+ ((Vector) (numberedID.elementAt(ind)))
										.elementAt(0);
						break;
					}
				}
				// System.out.println(temNode.getTimestep());
				// System.out.println(temNode.getID());
				// System.out.println(temNode.getX());
				// System.out.println(temNode.getY());

				LocX = temNode.getX();
				LocY = temNode.getY();

				// System.out.println("test " + LocY);

				if (LocX == 0.0 || LocX < 0.0) {
					LocX = 0.0005;
				} else if (LocX >= Integer.parseInt(topoLength)) {
					LocX = Integer.parseInt(topoLength) - 1;
				}
				if (LocY == 0.0 || LocY < 0.0) {
					LocY = 0.0005;
				} else if (LocY >= Integer.parseInt(topoWidth)) {
					LocY = Integer.parseInt(topoWidth) - 1;
				}

				// System.out.println("test " + LocY);

				/*
				 * String tempx = ""+temNode.getX(); String tempy =
				 * ""+temNode.getY();
				 * 
				 * if (tempx.equals("0.0")){ tempx = "0.005"; } else if
				 * (tempx.equals(topoWidth)){ tempx = "" + (temNode.getX()
				 * -1.0); } if (tempy.equals("0.0")){ tempy = "0.005"; } else if
				 * (tempy.equals(topoLength)){ tempy = "" + (temNode.getY()
				 * -1.0); }
				 */
				double tempVal = Double.parseDouble(temNode.getTimestep());
				localNodeTime = (int) tempVal;
				localNodeTime -= 1; // for synchronous

				/*
				 * //detect node off if (NodeCheckFlag[Integer.parseInt(temp)] ==
				 * 0) { //System.out.println("Hit!"); printer.print("$ns_ at "+
				 * localNodeTime +" \"$node_("+temp+") set X_
				 * "+temNode.getX()+"\"\n"); printer.print("$ns_ at "+
				 * localNodeTime +" \"$node_("+temp+") set Y_
				 * "+temNode.getY()+"\"\n"); printer.print("$ns_ at "+
				 * localNodeTime +" \"$node_("+temp+") set Z_ 0.0\"\n");
				 * NodeCheckFlag[Integer.parseInt(temp)] = 1; }
				 * 
				 * 
				 * if (localNodeTime > timeFlag ) { for (int ind = 0; ind <
				 * numberedID.size(); ind++) { if (NodeCheckFlag[ind] == 1 ) {
				 * //System.out.println(ind); NodeCountFlag[ind] += 1; if
				 * (NodeCountFlag[ind] >=3) { printer.print("$ns_ at "+
				 * localNodeTime +" \"$node_("+ind+") shutdown\"\n");
				 * NodeCheckFlag[ind] = 2; } } } timeFlag = localNodeTime; }
				 */

				// System.out.println(temp);
				NodeCountFlag[Integer.parseInt(temp)] = 0;
				// System.out.println(NodeCountFlag[Integer.parseInt(temp)]);

				// printer.print("$ns_ at "+ localNodeTime +" \"$node_("+temp+")
				// setdest "+
				// tempx+" "+tempy+" "+ temNode.getSpeed()+"\"\n");

				printer.print("$ns_ at " + localNodeTime + " \"$node_(" + temp
						+ ") setdest " + LocX + " " + LocY + " "
						+ temNode.getSpeed() + "\"\n");
			}
			/* Only the movements are required */
			if (onlyMovement == 1) {
				printer.close();
				return;
			}
			/*
			 * if(nodeGroups.isEmpty() == false){ for(int index=0; index<nodeGroups.size();
			 * index=index+2){ String groupID = (String) nodeGroups.get(index);
			 * String nodeID = (String) nodeGroups.get(index+1);
			 * 
			 * if(SRMGroup.containsKey(groupID)==false){ multiCastGroup m = new
			 * multiCastGroup(groupID); m.addMember(nodeID);
			 * SRMGroup.put(groupID,m); SRMGroupID.add(groupID); } else{
			 * multiCastGroup m = (multiCastGroup)SRMGroup.get(groupID);
			 * m.addMember(nodeID); SRMGroup.remove(groupID);
			 * SRMGroup.put(groupID,m); } }
			 * 
			 *  }
			 */
			// System.out.println(connections.size());
			if (connections.size() > 0) {
				printer.print("\n# Setup traffic flow between nodes\n");
				Vector vtemp;
				int agentNO = 0;
				int applicationNO = 0;
				int firstSCTP = 0;

				if (TCPFlag == 1) {
					printer.print("Agent/TCP set window_ " + tcpWindow_ + "\n");
					printer.print("Agent/TCP set packetSize_ " + tcpPacketSize_
							+ "\n");
					printer.print("Agent/TCP set maxburst_ " + tcpMaxburst_
							+ "\n");
					printer.print("Agent/TCP set maxcwnd_ " + tcpMaxcwnd_
							+ "\n");
				}
				// UDPFlag = 0;
				if (UDPFlag == 1) {
					// printer.print("Agent/UDP set window_ "+udpWindow_+"\n");
					printer.print("Agent/UDP set packetSize_ " + udpPacketSize_
							+ "\n");
					printer.print("Application/Traffic/CBR set rate_ "
							+ udpRate_ + "\n");
					// printer.print("Application/Traffic/CBR set interval_
					// "+udpInterval_+"\n");
					printer.print("Application/Traffic/CBR set random_ "
							+ udpRandom_ + "\n");
					printer.print("Application/Traffic/CBR set maxpkts_ "
							+ udpMaxpkts_ + "\n");
				}

				/*
				 * if(SRMFlag == 1){ printer.print("$ns_ at "+srmStop_+" \"$ns_
				 * clear-mcast\"\n");
				 * 
				 * printer.print("Agent/SRM set window_ "+srmWindow_+"\n");
				 * printer.print("Agent/SRM set packetSize_
				 * "+srmPacketSize_+"\n"); printer.print("Agent/SRM set
				 * burst_time_ "+srmBurst_time_+"\n"); printer.print("Agent/SRM
				 * set idle_time_ "+srmIdle_time_+"\n");
				 * printer.print("Agent/SRM set rate_ "+srmRate_+"\n"); }
				 */

//				System.out.println( " connections.size() - " + connections.size());
				for (int i = 0; i < connections.size(); i++) {
					// System.out.println(connSTime[i]);

					vtemp = (Vector) connections.elementAt(i);
					String temp2 = (String) vtemp.elementAt(0);
					String temp3 = (String) vtemp.elementAt(2);

					source = "";
					for (int ind = 0; ind < numberedID.size(); ind++) {
						if (temp2
								.equalsIgnoreCase((String) ((Vector) (numberedID
										.elementAt(ind))).elementAt(1))) {
							source = source
									+ ((Vector) (numberedID.elementAt(ind)))
											.elementAt(0);
							break;
						}
					}

					sink = "";
					for (int ind = 0; ind < numberedID.size(); ind++) {
						if (temp3
								.equalsIgnoreCase((String) ((Vector) (numberedID
										.elementAt(ind))).elementAt(1))) {
							sink = sink
									+ ((Vector) (numberedID.elementAt(ind)))
											.elementAt(0);
							break;
						}
					}

					String agentName = (String) vtemp.elementAt(4);
					if (agentName.compareTo("tcp") == 0) {

						printer.print("\nset " + "tcp" + agentNO
								+ " [new Agent/TCP]\n");
						printer.print("$tcp" + agentNO + " set class_ 2\n");
						printer.print("set sink" + agentNO
								+ " [new Agent/TCPSink]\n");
						printer.print("$ns_ attach-agent $node_(" + source
								+ ") $tcp" + agentNO + "\n");
						printer.print("$ns_ attach-agent $node_(" + sink
								+ ") $sink" + agentNO + "\n");
						printer.print("$ns_ connect $tcp" + agentNO + " $sink"
								+ agentNO + "\n");
						printer.print("set ftp" + applicationNO
								+ " [new Application/FTP]\n");
						printer.print("$ftp" + applicationNO
								+ " attach-agent $tcp" + agentNO + "\n");
						if (TCPFlag == 1) {
							printer.print("$ns_ at " + connSTime[i] + " \"$ftp"
									+ applicationNO + " start\"\n\n");
							printer.print("$ns_ at " + connETime[i] + " \"$ftp"
									+ applicationNO + " stop\"\n\n");
						} else
							printer.print("$ns_ at " + tcpStart_ + " \"$ftp"
									+ applicationNO + " start\"\n\n");

					} else if (agentName.compareTo("udp") == 0) {
						printer.print("\nset udp" + agentNO
								+ " [new Agent/UDP]\n");
						printer.print("$ns_ attach-agent $node_(" + source
								+ ") $udp" + agentNO + "\n");
						printer.print("set cbr" + applicationNO
								+ " [new Application/Traffic/CBR]\n");
						printer.print("$cbr" + applicationNO
								+ " attach-agent $udp" + agentNO + "\n");
						printer.print("set null" + agentNO
								+ " [new Agent/Null]\n");
						printer.print("$ns_ attach-agent $node_(" + sink
								+ ") $null" + agentNO + "\n");
						printer.print("$ns_ connect $udp" + agentNO + " $null"
								+ agentNO + "\n");
						if (UDPFlag == 1) {
							if (Double.parseDouble(udpStart_) < connSTime[i]) {
								printer.print("$ns_ at " + connSTime[i]
										+ " \"$cbr" + applicationNO
										+ " start\"\n\n");
							} else
								printer.print("$ns_ at " + udpStart_
										+ " \"$cbr" + applicationNO
										+ " start\"\n\n");
							printer.print("$ns_ at " + connETime[i] + " \"$cbr"
									+ applicationNO + " stop\"\n\n");
						} else
							printer.print("$ns_ at " + udpStart_ + " \"$cbr"
									+ applicationNO + " start\"\n\n");

					}
					agentNO++;
					applicationNO++;
				}
				/*
				 * if(nodeGroups.isEmpty() == false){ int SRMNumber=0;
				 * applicationNO = 0; //printer.print("$ns_ enableMcast\n");
				 * //printer.print("$ns_ multicast\n");
				 * 
				 * for(int s = 0; s < SRMGroupID.size();s++){ String groupID =
				 * (String) SRMGroupID.get(s); printer.print("set "+groupID+"
				 * [$ns_ allocaddr]\n"); printer.print("set srm"+SRMNumber+"
				 * [new Agent/SRM]\n"); printer.print("$srm"+SRMNumber+" set
				 * dst_ $"+groupID+"\n");
				 * 
				 * 
				 * multiCastGroup gEle = (multiCastGroup)SRMGroup.get(groupID);
				 * Vector mlist = gEle.getMember(); for(int k=0;k<mlist.size();k++){
				 * 
				 * String nodeID = (String) mlist.get(k);
				 * 
				 * String temp = ""; for (int ind = 0; ind < numberedID.size();
				 * ind++){ if
				 * (nodeID.equals((String)((Vector)(numberedID.elementAt(ind))).elementAt(1))){
				 * temp = temp +
				 * ((Vector)(numberedID.elementAt(ind))).elementAt(0); break; } }
				 * 
				 * 
				 * printer.print("$ns_ attach-agent $node_("+temp+")
				 * $srm"+SRMNumber+"\n"); //printer.print("$node_("+temp+")
				 * join-group $srm"+SRMNumber+" $"+groupID+"\n"); }
				 * printer.print("set exp"+applicationNO+" [new
				 * Application/Traffic/Exponential]\n");
				 * printer.print("$exp"+applicationNO+" attach-agent
				 * $srm"+SRMNumber+"\n"); printer.print("$srm"+SRMNumber+" log
				 * [open srmStats.tr w]\n"); printer.print("$srm"+SRMNumber+"
				 * trace [open srmEvents.tr w]\n");
				 * 
				 * if(SRMFlag == 1){ printer.print("$ns_ at "+ srmStart_ +"
				 * \"$srm"+SRMNumber+"start; $exp"+applicationNO+" start\"\n");
				 * 
				 * }else printer.print("$ns_ at "+ srmStart_ +"
				 * \"$srm"+SRMNumber+"start; $exp"+applicationNO+" start\"\n");
				 * 
				 * 
				 * 
				 * SRMNumber++; applicationNO++; } }
				 */

			}

			printer.print("#\n");
			printer.print("# Tell nodes when the simulation ends\n");
			printer.print("#\n");
			printer.print("for {set i 0} {$i < $val(nn) } {incr i} {\n");
			printer.print("    $ns_ at $stopTime \"$node_($i) reset\";\n");
			printer.print("}\n");
			printer.print("$ns_ at $stopTime \"stop\"\n");
			printer.print("$ns_ at $stopTime \"puts \\\"NS EXITING...\\\" ; $ns_ halt\"\n");
			printer.print("proc stop {} {\n");
			printer.print("    global ns_ tracefd");
			if (namflag == true)
				printer.print(" namtrace\n");
			else
				printer.print("\n");
			printer.print("    $ns_ flush-trace\n");
			printer.print("    close $tracefd\n");
			if (namflag == true)
				printer.print("    close $namtrace\n");
			printer.print("}\n\n");
			printer.print("puts \"Starting Simulation...\"\n");
			printer.print("$ns_ run\n\n");
			printer.close();

			connections.clear();
			conneStartTime.clear();
			return;
		}
	}
}
