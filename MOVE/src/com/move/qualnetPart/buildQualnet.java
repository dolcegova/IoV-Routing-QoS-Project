package com.move.qualnetPart;
import java.io.*;
import java.util.*;

import javax.swing.*;

import com.move.commonData.ReadDumpInfo;
import com.move.commonData.dumpData;
import com.move.commonData.edgeInfo;
import com.move.commonData.laneInfo;
import com.move.commonData.stepInfo;
import com.move.commonData.vehInfo;
import com.move.ns2Part.NSNode;

public class buildQualnet {

	private int maxVeh = 10000;
	
	String filename = "";
	String netFile = "";
	PrintWriter nodemob;
	PrintWriter nodepos;
	PrintWriter config;

	Vector initialNode = new Vector();
	Vector nodeMovement = new Vector();
	Vector numberedID = new Vector();

	/* Qualnet mobile nodes start from ID 1 */
	private int currid = 1;

	double xBoundary = 0.0;
	double yBoundary = 0.0;

	public String temptime = "";
	public String start = "";
	private int hasstart = 0;

	/* General Options */
	private String experiment = "";
	private String xboundary = "";
	private String yboundary = "";
	private String simtime = "";
	private String numnodes = "";
	private String routing = "";

	/* Physical layer options */
	private String physical = "";
	private String datarate = "";
	private String temperature = "";
	private String noisefactor = "";
	private String antennamodel = "";
	private String antennagain = "";
	private String antennaheight = "";
	private String antennaefficiency = "";
	private String antennamismatch = "";
	private String antennacableloss = "";
	private String antennaconnloss = "";

	/* Wireless options */
	private String frequency = "";
	private String proplimit = "";
	private String pathloss = "";
	private String fading = "";
	private String shadow = "";
	private String shadowmean = "";
	private String gaussianfile = "";
	private String maxvelocity = "";
	private String riceankfactor = "";

	/* Mac802.11 options */
	private String longpacketlimit = "";
	private String shortpacketlimit = "";
	private String rtsthreshold = "";
	private String propagationdelay = "";

	/* IP Network options */
	private String fragunit = "";
	private String ipqueuesize = "";
	private String loopback = "";

	/* TCL Options */
	private String tcp = "";
	private String delayack = "";
	private String delayshort = "";
	private String nagle = "";
	private String push = "";
	private String keepalive = "";
	private String sendbuffer = "";
	private String receivebuffer = "";
	private String maxsegmentsize = "";

	private JTable stattable;
	private Vector statVector = new Vector();

	public buildQualnet(String traceFile, String netXMLFile) {
		filename = traceFile;
		netFile = netXMLFile;
		stattable = new JTable();
	}

	public void setGeneral(String texperiment, String txboundary,
			String tyboundary, String tsimtime, String tnumnodes,
			String trouting) {
		experiment = texperiment;
		xboundary = txboundary;
		yboundary = tyboundary;
		simtime = tsimtime;
		numnodes = tnumnodes;
		routing = trouting;
	}

	public void setPhysical(String tphysical, String tdatarate,
			String ttemperature, String tnoisefactor, String tantennamodel,
			String tantennagain, String tantennaheight,
			String tantennaefficiency, String tantennamismatch,
			String tantennacableloss, String tantennaconnloss) {
		physical = tphysical;
		datarate = tdatarate;
		temperature = ttemperature;
		noisefactor = tnoisefactor;
		antennamodel = tantennamodel;
		antennagain = tantennagain;
		antennaheight = tantennaheight;
		antennaefficiency = tantennaefficiency;
		antennamismatch = tantennamismatch;
		antennacableloss = tantennacableloss;
		antennaconnloss = tantennaconnloss;
	}

	public void setWireless(String tfrequency, String tproplimit,
			String tpathloss, String tshadow, String tshadowmean,
			String tfading, String tgaussianfile, String tmaxvelocity,
			String triceankfactor) {
		frequency = tfrequency;
		proplimit = tproplimit;
		pathloss = tpathloss;
		shadow = tshadow;
		shadowmean = tshadowmean;
		fading = tfading;
		gaussianfile = tgaussianfile;
		maxvelocity = tmaxvelocity;
		riceankfactor = triceankfactor;
	}

	public void setMac(String tlongpacketlimit, String tshortpacketlimit,
			String trtsthreshold, String tpropagationdelay) {
		longpacketlimit = tlongpacketlimit;
		shortpacketlimit = tshortpacketlimit;
		rtsthreshold = trtsthreshold;
		propagationdelay = tpropagationdelay;
	}

	public void setIP(String tfragunit, String tipqueuesize, String tloopback) {
		fragunit = tfragunit;
		ipqueuesize = tipqueuesize;
		loopback = tloopback;
	}

	public void setTCL(String ttcp, String delayackflag, String delayshortflag,
			String nagleflag, String pushflag, String keepaliveflag,
			String tsendbuffer, String treceivebuffer, String tmaxsegmentsize) {
		tcp = ttcp;
		delayack = delayackflag;
		delayshort = delayshortflag;
		nagle = nagleflag;
		push = pushflag;
		keepalive = keepaliveflag;
		sendbuffer = tsendbuffer;
		receivebuffer = treceivebuffer;
		maxsegmentsize = tmaxsegmentsize;
	}

	public void addStat(String temp) {
		statVector.add(temp);
	}

	public void setStat(JTable tstattable) {
		stattable = tstattable;
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
		// System.out.println("Processing");
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
		String direction = " ";

		String temx1 = " ", temy1 = " ", temx2 = " ", temy2 = " ";
		String temlength = " ";
		String temposition = " ";
		String fromTemp = "";
		String toTemp = "";

		// This part parse XML.NET file.
		// List<String> variable uses same index in arraryList
		// Here are for edge information
		List<String> listEdge = new ArrayList<String>();
		List<String> listFrom = new ArrayList<String>();
		List<String> listTo = new ArrayList<String>();
		List<String> listLength = new ArrayList<String>();

		// Here are for node information
		List<String> listNode = new ArrayList<String>();
		List<String> listX = new ArrayList<String>();
		List<String> listY = new ArrayList<String>();

		// here are for current edge information
		String existVID[] = new String[maxVeh];
		String Current_source[] = new String[maxVeh];
		String Current_destination[] = new String[maxVeh];
		String Current_length[] = new String[maxVeh];

		for (int initial = 0; initial < maxVeh; initial++) {
			existVID[initial] = "";
			Current_source[initial] = "";
			Current_destination[initial] = "";
			Current_length[initial] = "";
		}

		boolean parseFlag = false, catchBoundary = false;

		File roadMap = new File(netFile);
		BufferedReader roadMapContent = new BufferedReader(new FileReader(
				roadMap), 65535);
		StringBuffer tempContent = new StringBuffer();
		String roadMapLine = roadMapContent.readLine();

		while (roadMapLine != null) {
			// System.out.println(roadMapLine);
			// road-map-line parse

			roadMapLine = clearSpace(roadMapLine);

			if (roadMapLine.equals("<net>"))
				parseFlag = true;

			// System.out.println(roadMapLine.length());
			// System.out.println(roadMapLine);

			if (roadMapLine.length() >= 7 && parseFlag == true) {
				StringTokenizer stringParse = new StringTokenizer(roadMapLine);
				// System.out.println(stringParse.countTokens());
				// ======================================== old version part
				// (sumo 0.10.3)
				// StringTokenizer stringParseType = new
				// StringTokenizer(roadMapLine, "-");
				// //System.out.println(stringParseType.nextToken());
				// if(catchBoundary == false &&
				// stringParseType.nextToken().equals("<conv") &&
				// stringParse.countTokens() == 1) {
				// // e.g.
				// <conv-boundary>0.00,0.00,1600.00,1600.00</conv-boundary>
				// //System.out.println(roadMapLine);
				// StringTokenizer stringParseForBoundaryX = new
				// StringTokenizer(roadMapLine, ",");
				// stringParseForBoundaryX.nextToken();
				// stringParseForBoundaryX.nextToken();
				// xBoundary = Double.parseDouble(
				// stringParseForBoundaryX.nextToken() );
				// StringTokenizer stringParseForBoundaryY = new
				// StringTokenizer(stringParseForBoundaryX.nextToken(), "<");
				// yBoundary = Double.parseDouble(
				// stringParseForBoundaryY.nextToken() );
				// catchBoundary = true;
				// //System.out.println(xBoundary);
				// //System.out.println(yBoundary);
				// }
				// ======================================== old version part
				// (sumo 0.10.3)

				StringTokenizer stringParseType = new StringTokenizer(
						roadMapLine);
				// System.out.println(stringParseType.nextToken());
				if (catchBoundary == false
						&& stringParseType.nextToken().equals("<location")
						&& stringParse.countTokens() >= 5) {
					// e.g. <location netOffset="0.00,0.00"
					// convBoundary="0.00,0.00,800.00,800.00"
					// origBoundary="0.00,0.00,800.00,800.00"
					// projParameter="!"/>
					// System.out.println(roadMapLine);
					StringTokenizer stringParseFindBoundary = new StringTokenizer(
							roadMapLine, " ");
					// System.out.println("show " +
					// stringParseFindBoundary.nextToken());
					stringParseFindBoundary.nextToken();
					stringParseFindBoundary.nextToken();
					StringTokenizer stringParseForBoundary = new StringTokenizer(
							stringParseFindBoundary.nextToken(), ",");
					stringParseForBoundary.nextToken();
					stringParseForBoundary.nextToken();
					// System.out.println("show " +
					// stringParseForBoundary.nextToken() );
					xBoundary = Double.parseDouble(stringParseForBoundary
							.nextToken());
					StringTokenizer stringParseForBoundaryY = new StringTokenizer(
							stringParseForBoundary.nextToken(), "\"");
					// System.out.println("show " +
					// stringParseForBoundaryY.nextToken() );
					yBoundary = Double.parseDouble(stringParseForBoundaryY
							.nextToken());
					// xBoundary = Double.parseDouble(
					// stringParseForBoundaryX.nextToken() );
					// StringTokenizer stringParseForBoundaryY = new
					// StringTokenizer(stringParseForBoundaryX.nextToken(),
					// "<");
					// yBoundary = Double.parseDouble(
					// stringParseForBoundaryY.nextToken() );
					catchBoundary = true;

					// System.out.println(xBoundary);
					// System.out.println(yBoundary);
					// System.in.read();
				}

				if (stringParse.countTokens() > 3) {
					String temp = stringParse.nextToken();
					// System.out.println(stringParse.countTokens());
					// System.out.println(temp);

					if (temp.compareTo("<edge") == 0
							&& stringParse.countTokens() == 6) {
						// System.out.println(" - > "+ temp);

						temp = stringParse.nextToken();
						// System.out.print(" target_1 " + getValue(temp));
						listEdge.add(getValue(temp));

						temp = stringParse.nextToken();
						// System.out.print(" target_2 " + getValue(temp));
						listFrom.add(getValue(temp));

						temp = stringParse.nextToken();
						// System.out.print(" target_3 " + getValue(temp) +
						// "\n");
						listTo.add(getValue(temp));

						// read the edge length
						roadMapLine = roadMapContent.readLine();
						roadMapLine = roadMapContent.readLine();
						roadMapLine = clearSpace(roadMapLine);
						StringTokenizer stringParse2 = new StringTokenizer(
								roadMapLine);
						String temp2 = stringParse2.nextToken();
						if (temp2.compareTo("<lane") == 0) {
							temp2 = stringParse2.nextToken();
							// System.out.println("1 " + getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println("2 " + getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println("3 " + getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println("4 " + getValue(temp2));

							// ======================================== old
							// version part (sumo 0.10.3)
							// temp2 = stringParse2.nextToken();
							// System.out.println("5 " + getValue(temp2));
							// ======================================== old
							// version part (sumo 0.10.3)

							listLength.add(getValue(temp2));
						}
						// System.in.read();
					}

					if (temp.compareTo("<junction") == 0
							&& stringParse.countTokens() >= 4) {
						temp = stringParse.nextToken();
						// System.out.println(getValue(temp));
						listNode.add(getValue(temp));

						temp = stringParse.nextToken();
						// System.out.println(getValue(temp));
						temp = stringParse.nextToken();
						// System.out.println(getValue(temp));
						listX.add(getValue(temp));

						temp = stringParse.nextToken();
						// System.out.println(getValue(temp));
						listY.add(getValue(temp));
						// System.in.read();
					}
				} else if (stringParse.countTokens() == 3) {
					String temp = stringParse.nextToken();
					// System.out.println(stringParse.countTokens());
					// System.out.println(temp);

					if (temp.compareTo("<edge") == 0) {
						// System.out.println(" - > "+ temp);

						temp = stringParse.nextToken();
						// System.out.print(" target_1 " + getValue(temp));
						listEdge.add(getValue(temp));
						listFrom.add(null);
						listTo.add(null);

						// read the edge length
						roadMapLine = roadMapContent.readLine();
						roadMapLine = roadMapContent.readLine();
						roadMapLine = clearSpace(roadMapLine);
						// System.out.println(roadMapLine);
						StringTokenizer stringParse2 = new StringTokenizer(
								roadMapLine);
						String temp2 = stringParse2.nextToken();
						if (temp2.compareTo("<lane") == 0) {
							temp2 = stringParse2.nextToken();
							// System.out.println(getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println(getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println(getValue(temp2));
							temp2 = stringParse2.nextToken();
							// System.out.println("++ " + getValue(temp2));
							listLength.add(getValue(temp2));
						}
						// System.in.read();
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

		HashMap map = new HashMap();
		int vehicle_counter = 0;

		for (int i = 0; i < allSteps.size(); i++) {
			// System.out.println("AllSteps = "+allSteps.size());
			stepInfo s = (stepInfo) allSteps.get(i);
			ArrayList allEdges = s.returnData();
			timeID = s.getID();

			// System.out.println("timeID-> " + timeID);

			for (int j = 0; j < allEdges.size(); j++) {
				// System.out.println("AllEdges = "+allEdges.size());
				edgeInfo e = (edgeInfo) allEdges.get(j);
				ArrayList allLanes = e.returnData();
				edgeID = e.getID();
				// System.out.println("edgeID-> " + edgeID);

				for (int m = 0; m < allLanes.size(); m++) {
					// System.out.println("AllLanes = "+allLanes.size());
					laneInfo l = (laneInfo) allLanes.get(m);
					ArrayList allVeh = l.returnData();
					laneID = l.getID();

					// System.out.println("laneID-> " + laneID);

					for (int n = 0; n < allVeh.size(); n++) {
						// System.out.println("AllVeh = "+allVeh.size());
						vehInfo v = (vehInfo) allVeh.get(n);
						vehID = v.getID();
						temposition = v.getPosition();
						speed = v.getSpeed();

						// System.out.println("vehID-> " + vehID);
						// System.out.println("speed-> " + speed);
						// System.out.println("temposition-> " + temposition);

						String tempID = "";

						// This part start process XML.NET file to find current
						// location of the vehicle

						int findEdge_index = 0;
						int findNode_index = 0;
						int findLocation_index = 0;
						boolean findNode_flag = false;
						boolean intersection_flag = false;
						int searchCounter = 0;
						String findTemp;

						temx1 = "";
						temy1 = "";
						temx2 = "";
						temy2 = "";

						// update current vehicle information
						for (int foundnode = 0; vehicle_counter != 0
								&& foundnode < maxVeh; foundnode++) {
							if (existVID[foundnode].compareTo(vehID) == 0) {
								findLocation_index = foundnode;
								findNode_flag = true;
								// System.out.println("!! find exist vehicle ->"
								// + vehID + " index -> " + findLocation_index);
								break;
							}
						}

						if (findNode_flag == false) {
							existVID[vehicle_counter] = vehID;
							// System.out.println("cannot find exist vehicle ->"
							// + vehID + " index -> " + vehicle_counter);
						}

						for (String findEdge : listEdge) {
							findTemp = listEdge.get(findEdge_index);
							if (findTemp.compareTo(edgeID) == 0) {
								fromTemp = listFrom.get(findEdge_index);
								toTemp = listTo.get(findEdge_index);
								temlength = listLength.get(findEdge_index);

								// the vehicle is moving on intersection
								if (fromTemp != null) {
									if (findNode_flag == true) {
										Current_source[findLocation_index] = fromTemp;
										Current_destination[findLocation_index] = toTemp;
										Current_length[findLocation_index] = temlength;
										// System.out.println(" --> index " +
										// findLocation_index);
									} else {
										Current_source[vehicle_counter] = fromTemp;
										Current_destination[vehicle_counter] = toTemp;
										Current_length[vehicle_counter] = temlength;
										// System.out.println(" --> index " +
										// vehicle_counter);
										vehicle_counter++;
									}
								} else {
									intersection_flag = true;
									fromTemp = Current_source[findLocation_index];
									toTemp = Current_destination[findLocation_index];
								}

								// System.out.print("edgeID -> " + findTemp + "
								// ");
								// System.out.print(fromTemp + " " + toTemp + "
								// ");
								// System.out.println(" L-> " + temlength +
								// "\n");

								for (String findNode : listNode) {
									findTemp = listNode.get(findNode_index);
									if (findTemp.compareTo(fromTemp) == 0) {

										temx1 = listX.get(findNode_index);
										temy1 = listY.get(findNode_index);
										searchCounter++;
									}

									if (findTemp.compareTo(toTemp) == 0) {

										temx2 = listX.get(findNode_index);
										temy2 = listY.get(findNode_index);
										searchCounter++;
									}

									if (searchCounter >= 2)
										break;
									findNode_index++;
								}
								// System.out.print(" " + temx1 + " " + temy1);
								// System.out.print(" " + temx2 + " " + temy2 +
								// "\n\n");
							}
							findEdge_index++;
						}

						Double xfrom, yfrom, xto, yto, position, length;
						Double extra_pos, extra_len;
						double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
						double ratio = 0.0;

						double a = 0.0, b = 0.0;

						xfrom = new Double(temx1);
						xto = new Double(temx2);

						yfrom = new Double(temy1);
						yto = new Double(temy2);

						// if (xto < 0 || yto < 0 || xfrom < 0 || yfrom < 0) {
						// System.out.println("x from = " + xfrom + " x to =" +
						// xto + " y from = " + yfrom + " y to = " + yto);
						// }

						// if (xfrom.doubleValue() > xBoundary)
						// xBoundary = xfrom.doubleValue();
						// if (xto.doubleValue() > xBoundary)
						// xBoundary = xto.doubleValue();
						// if (yfrom.doubleValue() > yBoundary)
						// yBoundary = yfrom.doubleValue();
						// if (yto.doubleValue() > yBoundary)
						// yBoundary = yto.doubleValue();

						position = new Double(temposition);
						length = new Double(temlength);

						if (intersection_flag == true) {
							extra_pos = new Double(
									Current_length[findLocation_index]);
							extra_len = new Double(
									Current_length[findLocation_index]);
							position += extra_pos;
							length += extra_len;
						}

						// System.out.println("pos =" + position + " length =" +
						// length);

						// if (vehID.compareTo("flow10A_0") == 0)
						// System.in.read();

						if ((xfrom.doubleValue() < xto.doubleValue())
								&& (yfrom.doubleValue() != yto.doubleValue())) {
							// System.out.println("1");
							x1 = xfrom.doubleValue();
							y1 = yfrom.doubleValue();

							x2 = xto.doubleValue();
							y2 = yto.doubleValue();

							ratio = position.doubleValue()
									/ length.doubleValue();

							a = x1 - ratio * (x1 - x2);

							if (y2 > y1) {
								b = y2 - ratio * (y2 - y1);
								direction = "southwest-to-northeast";
							} else {
								b = y1 - ratio * (y1 - y2);
								direction = "northwest-to-southeast";
							}

						} else if ((xfrom.doubleValue() > xto.doubleValue())
								&& (yfrom.doubleValue() != yto.doubleValue())) {
							// System.out.println("2 pos = " + position + " x
							// from = " + xfrom + " x to = " + xto + " y from =
							// " + yfrom + " y to = " + yto);
							x1 = xto.doubleValue();
							y1 = yto.doubleValue();

							x2 = xfrom.doubleValue();
							y2 = yfrom.doubleValue();

							ratio = position.doubleValue()
									/ length.doubleValue();

							a = x2 - ratio * (x2 - x1);

							if (y2 > y1) {
								b = y2 - ratio * (y2 - y1);
								direction = "northeast-to-southwest";
							} else {
								b = y1 - ratio * (y1 - y2);
								direction = "southeast-to-northwest";
							}
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
						}
						if (map.isEmpty()) {
							// System.out.println(a+ " " + b);
							NSNode tempNode = new NSNode(timeID, vehID, a, b);
							map.put(vehID, tempNode);
							initialNode.add(tempNode);
							// System.out.println(timeID);

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
					}
				}
			}
		}
		// System.out.println("over");
		long ProcessTime = System.currentTimeMillis() - StartTime;
		System.out.println("Total Tcl translation time (ms) = " + ProcessTime);
		d = null;
		r = null;
		allSteps = null;
		existVID = null;
		Current_source = null;
		Current_destination = null;
	}

	public void write(String configfile) {
		try {
			nodepos = new PrintWriter(new BufferedWriter(new FileWriter(
					configfile + ".nodes")));
		} catch (IOException e) {
		}
		;
		for (int index = 0; index < initialNode.size(); index++) {
			NSNode tempNode = (NSNode) initialNode.get(index);
			String temp = "";
			for (int ind = 0; ind < numberedID.size(); ind++) {
				if (tempNode.getID().equals(
						(String) ((Vector) (numberedID.elementAt(ind)))
								.elementAt(1))) {
					temp = temp
							+ ((Vector) (numberedID.elementAt(ind)))
									.elementAt(0);
					break;
				}
			}
			/*
			 * Forcing the initial nodes to have timestamp of 1 This is to
			 * overcome Qualnet strange requirement, which is to set all the
			 * nodes initial position at timestamp 1
			 */
			nodepos.print(temp + " " + "1" + " (" + tempNode.getX() + " "
					+ tempNode.getY() + " 0.0) 0.0 0.0\n");
		}
		for (int index = 0; index < nodeMovement.size(); index++) {
			NSNode tempNode = (NSNode) nodeMovement.get(index);
			String temp = "";
			for (int ind = 0; ind < numberedID.size(); ind++) {

				if (tempNode.getID().equals(
						(String) ((Vector) (numberedID.elementAt(ind)))
								.elementAt(1))) {
					temp = temp
							+ ((Vector) (numberedID.elementAt(ind)))
									.elementAt(0);
					break;
				}
			}
			nodepos.print(temp + " " + tempNode.getTimestep() + " ("
					+ tempNode.getX() + " " + tempNode.getY()
					+ " 0.0) 0.0 0.0\n");
		}
		nodepos.close();

		try {
			nodemob = new PrintWriter(new BufferedWriter(new FileWriter(
					configfile + ".mobility")));
		} catch (IOException e) {
		}
		;
		for (int index = 0; index < nodeMovement.size(); index++) {
			NSNode tempNode = (NSNode) nodeMovement.get(index);
			String temp = "";
			for (int ind = 0; ind < numberedID.size(); ind++) {

				if (tempNode.getID().equals(
						(String) ((Vector) (numberedID.elementAt(ind)))
								.elementAt(1))) {
					temp = temp
							+ ((Vector) (numberedID.elementAt(ind)))
									.elementAt(0);
					break;
				}
			}
			nodemob.print(temp + " " + tempNode.getTimestep() + " ("
					+ tempNode.getX() + " " + tempNode.getY()
					+ " 0.0) 0.0 0.0\n");
		}
		nodemob.close();

		try {
			config = new PrintWriter(new BufferedWriter(new FileWriter(
					configfile + ".config")));
		} catch (IOException e) {
		}
		;
		config.print("#*************General*************\n");
		config.print("VERSION 4.0\n");
		config.print("EXPERIMENT-NAME " + experiment + "\n");
		config.print("SIMULATION-TIME " + simtime + "S\n");
		config.print("SEED 1\n\n");

		config.print("#*************Parallel Settings***********\n");
		config.print("PARTITION-SCHEME AUTO\n\n");

		config.print("#*************Terrain**********\n");
		config.print("COORDINATE-SYSTEM CARTESIAN\n");
		config.print("TERRAIN-DIMENSIONS ( " + xboundary + ", " + yboundary
				+ " }\n");
		config.print("DUMMY-ALTITUDES ( 1500, 1500 )\n");
		config.print("TERRAIN-DATA-BONUDARY-CHECK YES\n\n");

		config.print("#**************Node Positioning***********\n");
		config.print("#**************Nodes***********\n");
		config.print("DUMMY-NUMBER-OF-NODES " + numnodes + "\n");
		config.print("NODE-PLACEMENT FILE\n");
		config.print("NODE-POSITION-FILE ");
		config.print(configfile + ".nodes\n");
		config.print("#**************Mobility***********\n");
		config.print("MOBILITY FILE\n");
		config.print("DUMMY-MOBILITY-FILE ");
		config.print(configfile + ".mobility" + "\n");
		config.print("MOBILITY-POSITION-GRANULARITY 5.0\n");
		config.print("MOBILITY-GROUND-NODE NO\n\n");

		config
				.print("#********************Wireless Settings****************\n");
		config.print("#*********************Channel*****************\n");
		config.print("PROPAGATION-CHANNEL-FREQUENCY " + frequency + "\n");
		config.print("PROPAGATION-MODEL STATISTICAL\n");
		config.print("PROPAGATION-LIMIT " + proplimit + "\n");
		config.print("PROPAGATION-PATHLOSS-MODEL " + pathloss + "\n");
		config.print("PROPAGATION-SHADOWING-MODEL " + shadow + "\n");

		if (shadow.equals("CONSTANT") || shadow.equals("LOGNORMAL")) {
			config.print("PROPAGATION-SHADOWING-MEAN " + shadowmean + "\n");

		}
		config.print("PROPAGATION-FADING-MODEL " + fading + "\n");

		if (fading.equals("RAYLEIGH")) {
			config.print("PROPAGATION-FADING-GAUSSIAN-COMPONENTS-FILE ");
			config.print(gaussianfile + "\n");
			config.print("PROPAGATION-FADING-MAX-VELOCITY " + maxvelocity
					+ "\n");
		} else if (fading.equals("RICEAN")) {
			config.print("PROPAGATION-FADING-GAUSSIAN-COMPONENTS-FILE ");
			config.print(gaussianfile + "\n");
			config.print("PROPAGATION-FADING-MAX-VELOCITY " + maxvelocity
					+ "\n");
			config.print("PROPAGATION-RICEAN-K-FACTOR " + riceankfactor
					+ "\n\n");
		}

		config.print("#****************Radio/Physical Layer***********\n");
		config.print("PHY-MODEL PHY" + physical + "\n");
		config.print("PHY802.11-AUTO-RATE-FALLBACK NO\n");
		config.print("PHY802.11-DATA-RATE " + datarate + "\n");
		config.print("PHY802.11-DATA-RATE-FOR-BROADCAST " + datarate + "\n");

		if (physical.equals("802.11a")) {
			config.print("PHY802.11a-TX-POWER--6MBPS  20.0\n");
			config.print("PHY802.11a-TX-POWER--9MBPS  20.0\n");
			config.print("PHY802.11a-TX-POWER-12MBPS  19.0\n");
			config.print("PHY802.11a-TX-POWER-18MBPS  19.0\n");
			config.print("PHY802.11a-TX-POWER-24MBPS  18.0\n");
			config.print("PHY802.11a-TX-POWER-36MBPS  18.0\n");
			config.print("PHY802.11a-TX-POWER-48MBPS  16.0\n");
			config.print("PHY802.11a-TX-POWER-54MBPS  16.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY--6MBPS  -85.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY--9MBPS  -85.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-12MBPS  -83.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-18MBPS  -83.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-24MBPS  -78.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-32MBPS  -78.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-48MBPS  -69.0\n");
			config.print("PHY802.11a-RX-SENSITIVITY-54MBPS  -69.0\n");
		} else if (physical.equals("802.11b")) {
			config.print("PHY802.11b-TX-POWER--1MBPS  15.0\n");
			config.print("PHY802.11b-TX-POWER--2MBPS  15.0\n");
			config.print("PHY802.11b-TX-POWER--6MBPS  15.0\n");
			config.print("PHY802.11b-TX-POWER-11MBPS  15.0\n");
			config.print("PHY802.11b-RX-SENSITIVITY--1MBPS  -93.0\n");
			config.print("PHY802.11b-RX-SENSITIVITY--2MBPS  -89.0\n");
			config.print("PHY802.11b-RX-SENSITIVITY--6MBPS  -87.0\n");
			config.print("PHY802.11b-RX-SENSITIVITY-11MBPS  -83.0\n");
		}

		config.print("PHY802.11-ESTIMATED-DIRECTIONAL-ANTENNA-GAIN 15.0\n");
		config.print("PHY-RX-MODEL PHY" + physical + "\n");
		config.print("PHY-LISTENABLE-CHANNEL-MASK 1\n");
		config.print("PHY-LISTENING-CHANNEL-MASK 1\n");
		config.print("PHY-TEMPERATURE " + temperature + "\n");
		config.print("PHY-NOISE-FACTOR " + noisefactor + "\n");
		config.print("ANTENNA-MODEL " + antennamodel + "\n");
		config.print("ANTENNA-GAIN " + antennagain + "\n");
		config.print("ANTENNA-HEIGHT " + antennaheight + "\n");
		config.print("ANTENNA-EFFICIENCY " + antennaefficiency + "\n");
		config.print("ANTENNA-MISMATCH-LOSS " + antennamismatch + "\n");
		config.print("ANTENNA-CABLE-LOSS " + antennacableloss + "\n");
		config.print("ANTENNA-CONNECTION-LOSS " + antennaconnloss + "\n\n");

		config.print("#******************MAC Protocol***************\n");
		config.print("MAC-PROTOCOL MAC802.11\n");
		config.print("MAC-802.11-DIRECTIONAL-ANTENNA-MODE NO\n");
		config.print("MAC-802.11-SHORT-PACKET-TRANSMIT-LIMIT "
				+ shortpacketlimit + "\n");
		config.print("MAC-802.11-LONG-PACKET-TRANSMIT-LIMIT " + longpacketlimit
				+ "\n");
		config.print("MAC-802.11-RTS-THRESHOLD " + rtsthreshold + "\n");
		config.print("MAC-802.11-PCF-STATISTICS NO\n");
		config.print("MAC-PROPAGATION-DELAY " + propagationdelay + "\n");
		config.print("PROMISCUOS-MODE YES\n");

		config.print("#*****************Adaptation Protocols*************\n");
		config.print("# ************* Adaptation Layer ***********\n");
		config.print("ADAPTATION-LAYER-STATISTICS NO\n");
		config.print("ATM-STATIC-ROUTE NO\n\n");

		config.print("# ************* ARP Specific ***********\n");
		config.print("# ************* ARP Enabled ***********\n");
		config.print("ARP-ENABLED NO\n");
		config.print("# ************* ARP Specs ***********\n");
		config.print("ARP-TIMEOUT-INTERVAL 20M\n\n");

		config.print("# ************* Network Protocols ***********\n");
		config.print("# ************* Network Protocol ***********\n");
		config.print("NETWORK-PROTOCOL IP\n");
		config.print("IP-ENABLE-LOOPBACK YES\n");
		config.print("IP-LOOPBACK-ADDRESS " + loopback + "\n");
		config.print("IP-FRAGMENTATION-UNIT " + fragunit + "\n");
		config.print("IP-QUEUE-NUM-PRIORITIES 3\n");
		config
				.print("IP-QUEUE-PRIORITY-INPUT-QUEUE-SIZE " + ipqueuesize
						+ "\n");
		config.print("DUMMY-PRIORITY-QUEUE-SIZE NO\n");
		config.print("IP-QUEUE-PRIORITY-QUEUE-SIZE " + ipqueuesize + "\n");
		config.print("DUMMY-PRIORITY-WISE-IP-QUEUE-TYPE NO\n");
		config.print("IP-QUEUE-TYPE FIFO\n");
		config.print("ECN NO\n");
		config.print("IP-QUEUE-SCHEDULER STRICT-PRIORITY\n");
		config.print("ROUTER-BACKPLANE-THROUGHPUT 0\n\n");

		config.print("# ************* Routing Protocol ***********\n");
		config.print("DUMMY-ROUTING DYNAMIC\n");
		config.print("ROUTING-PROTOCOL " + routing + "\n");
		config.print("HSRP-PROTOCOL NO\n");
		config.print("STATIC-ROUTE NO\n");
		config.print("DEFAULT-ROUTE NO\n\n");

		config.print("# ************* MPLS configuration ***********\n");
		config.print("MPLS-PROTOCOL NO\n\n");

		config.print("# ************* Transport Layer ***********\n");
		config.print("TCP " + tcp + "\n");

		if (tcp.equals("RENO"))
			config.print("TCP-USE-RFC1323 YES\n");
		else
			config.print("TCP-USE-RFC1323 NO\n");

		config.print("TCP-DELAY-ACKS " + delayack + "\n");
		config.print("TCP-DELAY-SHORT-PACKETS-ACKS " + delayshort + "\n");
		config.print("TCP-USE-NAGLE-ALGORITHM " + nagle + "\n");
		config.print("TCP-USE-KEEPALIVE-PROBES " + keepalive + "\n");
		config.print("TCP-USE-PUSH " + push + "\n");
		config.print("TCP-MSS " + maxsegmentsize + "\n");
		config.print("TCP-SEND-BUFFER " + sendbuffer + "\n");
		config.print("TCP-RECEIVE-BUFFER " + receivebuffer + "\n\n");

		config.print("# ************* ATM Layer2 ***********\n");
		config.print("ATM-RED-MIN-THRESHOLD 5\n");
		config.print("ATM-RED-MAX-THRESHOLD 15\n");
		config.print("ATM-RED-MAX-PROBABILITY 0.02\n");
		config.print("ATM-RED-SMALL-PACKET-TRANSMISSION-TIME 10MS\n");
		config.print("ATM-QUEUE-SIZE 15000\n");
		config.print("ATM-SCHEDULER-STATISTICS NO\n");
		config.print("ATM-LAYER2-STATISTICS NO\n");
		config.print("ATM-QUEUE-STATISTICS NO\n\n");

		config.print("# ************* Traffic and Status ***********\n");
		config.print("# ************* Application Layer ***********\n");
		config.print("APP-CONFIG-FILE " + configfile + ".app\n");

		config.print("# ************* Extras ***********\n");
		config.print("# ************* Tracing ***********\n");
		config.print("PACKET-TRACE NO\n");
		config.print("ACCESS-LIST-TRACE NO\n\n");

		config.print("# ************* Statistics ***********\n");
		config.print("# ************* Statistics ***********\n");

		config.print("APPLICATION-STATISTICS " + (String) statVector.get(0)
				+ "\n");
		config.print("TCP-STATISTICS " + (String) statVector.get(1) + "\n");
		config.print("ROUTING-STATISTICS " + (String) statVector.get(2) + "\n");
		config.print("IGMP-STATISTICS " + (String) statVector.get(3) + "\n");
		config.print("EXTERIOR-GATEWAY-PROTOCOL-STATISTICS "
				+ (String) statVector.get(4) + "\n");
		config.print("NETWORK-LAYER-STATISTICS " + (String) statVector.get(5)
				+ "\n");
		config.print("QUEUE-STATISTICS " + (String) statVector.get(6) + "\n");
		config.print("MAC-LAYER-STATISTICS " + (String) statVector.get(7)
				+ "\n");
		config.print("PHY-LAYER-STATISTICS " + (String) statVector.get(8)
				+ "\n");
		config
				.print("MOBILITY-STATISTICS " + (String) statVector.get(9)
						+ "\n");
		config.print("RSVP-STATISTICS " + (String) statVector.get(10) + "\n");
		config.print("DIFFSERV-EDGE-ROUTER-STATISTICS "
				+ (String) statVector.get(11) + "\n");
		config.print("ACCESS-LIST-STATISTICS " + (String) statVector.get(12)
				+ "\n");
		config.print("UDP-STATISTICS NO\n");

		config.print("\n");

		/*
		 * config.print("APPLICATION-STATISTICS YES\n");
		 * config.print("TCP-STATISTICS YES\n"); config.print("UDP-STATISTICS
		 * YES\n"); config.print("RSVP-STATISTICS NO\n");
		 * config.print("ROUTING-STATISTICS YES\n");
		 * config.print("ACCESS-LIST-STATISTICS NO\n");
		 * config.print("ROUTE-REDISTRIBUTION-STATISTICS NO\n");
		 * config.print("IGMP-STATISTICS NO\n");
		 * config.print("EXTERIOR-GATEWAY-PROTOCOL-STATISTICS YES\n");
		 * config.print("NETWORK-LAYER-STATISTICS YES\n");
		 * config.print("DIFFSERV-EDGE-ROUTER-STATISTICS NO\n");
		 * config.print("QUEUE-STATISTICS YES\n");
		 * config.print("MAC-LAYER-STATISTICS YES\n");
		 * config.print("PHY-LAYER-STATISTICS YES\n");
		 * config.print("MOBILITY-STATISTICS NO\n");
		 */

		config.print("# ************* Node Specific ***********\n\n");
		config.print("# ************* Device properties ***********\n\n");
		config.print("# ************* Router Specs ***********\n");
		config
				.print("# ************* Router Configuration Specs ***********\n\n");
		config.print("# ************* Node Orientation ***********\n");
		config.print("AZIMUTH 0\n");
		config.print("ELEVATION 0\n\n");

		config.print("# ************* Parallel Properties ***********\n");
		config.print("PARTITION 0\n\n");

		config.print("#----------------Default Subnet -----------------\n");
		config.print("SUBNET N8-192.0.0.0 { 1 thru " + numnodes
				+ " } Default\n\n");

		for (int index = 1; index <= initialNode.size(); index++) {
			NSNode tempNode = (NSNode) initialNode.get(index - 1);

			String temp = "";
			for (int ind = 0; ind < numberedID.size(); ind++) {
				if (tempNode.getID().equals(
						(String) ((Vector) (numberedID.elementAt(ind)))
								.elementAt(1))) {
					temp = temp
							+ ((Vector) (numberedID.elementAt(ind)))
									.elementAt(0);
					break;
				}
			}

			config.print("[" + index + "] HOSTNAME Mobile " + temp + "\n");
		}

		config.print("\n");
		config.print("[ 1 thru " + numnodes + " ] MOBILITY FILE\n\n");
		config.print("IP-FORWARDING YES\n");
		config.print("[ 1 thru " + numnodes + " ] IP-FORWARDING YES\n\n");

		config.close();

	}
}
