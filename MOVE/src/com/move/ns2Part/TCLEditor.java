package com.move.ns2Part;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.JaveFileFilter;

import java.awt.*;
import java.awt.event.*;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney

revised by Chien-Ming Chou
Taiwan, NCKU, CSIE

2009/02/10
*/

public class TCLEditor extends JFrame {

	private String filename = "untitled";
	private String sumoFile = "untitled";
	private String netFile = "untitled";

	public boolean mobileflag = false;
	public boolean namflag = false;
	public String UDPnoise = "NO";

	private JComboBox transport = new JComboBox();

	private String flag1 = "OFF";
	private String flag2 = "OFF";
	private String flag3 = "OFF";
	private String flag4 = "OFF";
	static JFileChooser chooser;
	private static int saveflag = 0;

	private static int macflag = 0;
	private static int agentflag = 0;
	private static int movementflag = 0;
	private static int routerflag = 0;
	private static int hasimported = 0;
	
	private ButtonGroup buttonGroup1;
	
	private buildNS ns;

	private JCheckBox onlyMovementCheck;

	private NodesDynamicTableModel mobnodestable = new NodesDynamicTableModel();
	private ConnectionsDynamicTableModel connstable = new ConnectionsDynamicTableModel();

	private JTextField TCPmaxburst;
	private JTextField TCPmaxcwnd;
	private JTextField TCPpacketsize;
	private JTextField TCPstarttime;
	private JTextField TCPstoptime;
	private JTextField TCPwindowsize;

	private JTextField UDPMaxPacket;
	private JCheckBox UDPRandomCheck;
	private JTextField UDPRate;
	private JTextField UDPInterval;
	private JTextField UDPpacketsize;
	private JTextField UDPstarttime;
	private JTextField UDPstoptime;
	private JTextField ConnectionTime;
	private JTextField StartConnectionTime;
	private JTextField RandConnectionTime;	
	//private JTextField UDPwindowsize;

	private JMenu FileMenu;
	private JMenu HelpMenu;
	private JMenuItem NewMenu;
	private JMenuItem QuitMenu;
	private JMenuItem SaveAsMenu;
	private JMenuItem SaveMenu;
	
	private JToggleButton addConnectionButton;
	private JToggleButton addUDP;
	private JToggleButton addTCP;
	private JToggleButton autoAssign;
	private JTextField assignNo;
	
	private JTextField adhocProtocol;
	private JCheckBox agenTraceCheck;
	private JTextField antennaModel;
	private JTextField channelType;
	private JTextField ifqType;
	private JMenuItem importSUMOMenu;

	private JLabel jLabel1;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel12;
	private JLabel jLabel13;
	private JLabel jLabel14;
	private JLabel jLabel15;
	private JLabel jLabel2;
	private JLabel jLabel20;
	private JLabel jLabel21;
	private JLabel jLabel22;
	private JLabel jLabel23;
	private JLabel jLabel25;
	private JLabel jLabel26;
	private JLabel jLabel28;
	private JLabel jLabel29;
	private JLabel jLabel3;
	private JLabel jLabel30;
	private JLabel jLabel31;
	private JLabel jLabel35;
	private JLabel jLabel38;
	private JLabel jLabel4;
	private JLabel jLabel40;
	private JLabel jLabel41;
	private JLabel jLabel42;
	private JLabel jLabel43;
	private JLabel jLabel44;
	private JLabel jLabel45;
	private JLabel jLabel46;
	private JLabel jLabel47;
	private JLabel jLabel48;
	private JLabel jLabel49;
	private JLabel jLabel5;
	private JLabel jLabel50;
	private JLabel jLabel51;
	private JLabel jLabel52;
	private JLabel jLabel53;
	private JLabel jLabel54;
	private JLabel jLabel55;
	private JLabel jLabel56;
	private JLabel jLabel57;
	private JLabel jLabel6;
	private JLabel jLabel7;
	private JLabel jLabel8;
	private JLabel jLabel9;
	private JMenuBar jMenuBar1;
	private JScrollPane jScrollPane1;
	private JScrollPane jScrollPane2;
	private JTable jTable1;
	private JTable jTable2;
	private JTextField llType;
	private JCheckBox macTraceCheck;
	private JTextField macType;
	private JTextField maxIFQ;

	private JTextField mobileNodesNo;
	private JCheckBox movementTraceCheck;
	private JTextField namPath;
	private JCheckBox namTraceCheck;
	private JTextField nicType;
	private JButton openNamButton;
	private JMenuItem quickHelpMenu;
	private JTextField radioModel;
	private JButton removeConnectionButton;
	private JCheckBox routerTraceCheck;

	private JTextField stopTime;
	private JTextField tracePath;
	private JButton tracePathButton;
	private JTextField xBoundary;
	private JTextField yBoundary;
	
	private int onlyMovement = 0;	
	
	//for autoconnection assign
	private int connNo = 0;
	
	public TCLEditor() {
		setSize(1000, 800);
	    setDefaultCloseOperation(EXIT_ON_CLOSE);
	    init();
	    setVisible(true);
	}

	private void init() {		
		GridBagConstraints gridBagConstraints;		
		getContentPane().setLayout(new GridBagLayout());		
//		getContentPane().setBackground(Color.WHITE);
			
		jLabel1 = new JLabel();
		jLabel23 = new JLabel();
		jLabel2 = new JLabel();
		channelType = new JTextField();
		jLabel4 = new JLabel();
		nicType = new JTextField();
		jLabel5 = new JLabel();
		ifqType = new JTextField();
		jLabel8 = new JLabel();
		antennaModel = new JTextField();
		jLabel12 = new JLabel();
		adhocProtocol = new JTextField();
		jLabel6 = new JLabel();
		radioModel = new JTextField();
		jLabel7 = new JLabel();
		macType = new JTextField();
		jLabel10 = new JLabel();
		maxIFQ = new JTextField();
		jLabel9 = new JLabel();
		llType = new JTextField();
		jLabel22 = new JLabel();
		jScrollPane1 = new JScrollPane();
		jTable1 = new JTable();
		jLabel13 = new JLabel();
		jLabel14 = new JLabel();
		xBoundary = new JTextField();
		jLabel15 = new JLabel();
		yBoundary = new JTextField();
		jLabel20 = new JLabel();
		stopTime = new JTextField();
		jLabel11 = new JLabel();
		mobileNodesNo = new JTextField();
		macTraceCheck = new JCheckBox();
		namTraceCheck = new JCheckBox();
		agenTraceCheck = new JCheckBox();
		routerTraceCheck = new JCheckBox();
		movementTraceCheck = new JCheckBox();
		jLabel25 = new JLabel();
		namPath = new JTextField();
		openNamButton = new JButton();
		jLabel21 = new JLabel();
		tracePath = new JTextField();
		tracePathButton = new JButton();
		onlyMovementCheck = new JCheckBox();
		jLabel3 = new JLabel();
		jScrollPane2 = new JScrollPane();
		jTable2 = new JTable();
		addConnectionButton = new JToggleButton();
		addUDP = new JToggleButton();
		addTCP = new JToggleButton();
		autoAssign = new JToggleButton();
		assignNo = new JTextField();
		removeConnectionButton = new JButton();
		jLabel28 = new JLabel();
		jLabel29 = new JLabel();
		jLabel30 = new JLabel();		
		jLabel26 = new JLabel();
		jLabel31 = new JLabel();
		jLabel40 = new JLabel();
		UDPpacketsize = new JTextField();
		jLabel43 = new JLabel();
		//UDPwindowsize = new JTextField();
		jLabel41 = new JLabel();
		UDPstarttime = new JTextField();
		ConnectionTime = new JTextField();
		StartConnectionTime = new JTextField();
		RandConnectionTime = new JTextField();
		jLabel42 = new JLabel();
		jLabel43 = new JLabel();
		UDPstoptime = new JTextField();
		jLabel35 = new JLabel();
		UDPRate = new JTextField();
		UDPInterval = new JTextField();
		jLabel38 = new JLabel();
		UDPMaxPacket = new JTextField();
		UDPRandomCheck = new JCheckBox();
		jLabel46 = new JLabel();
		TCPpacketsize = new JTextField();
		jLabel47 = new JLabel();
		TCPwindowsize = new JTextField();
		jLabel50 = new JLabel();
		TCPstarttime = new JTextField();
		jLabel49 = new JLabel();
		TCPstoptime = new JTextField();
		jLabel44 = new JLabel();
		TCPmaxburst = new JTextField();
		jLabel45 = new JLabel();
		TCPmaxcwnd = new JTextField();
		jMenuBar1 = new JMenuBar();
		FileMenu = new JMenu();
		NewMenu = new JMenuItem();
		importSUMOMenu = new JMenuItem();
		SaveMenu = new JMenuItem();
		SaveAsMenu = new JMenuItem();
		QuitMenu = new JMenuItem();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();		
		buttonGroup1 = new ButtonGroup();		
		
		if(chooser == null) {
			chooser = new JFileChooser("Select The File");
		}
		
		setTitle("NS-2 Script Generator");
		jLabel1.setFont(new Font("Arial", 1, 24));
		jLabel1.setForeground(new Color(255, 0, 51));
		jLabel1.setHorizontalAlignment(SwingConstants.LEFT);
		jLabel1.setText("Static Traffic Model Generator for NS-2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 12;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 0, 10, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		jLabel23.setFont(new Font("Tahoma", 1, 11));
		jLabel23.setForeground(new Color(0, 51, 255));
		jLabel23.setText("General Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel23, gridBagConstraints);

		jLabel2.setText("Channel Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel2, gridBagConstraints);

		channelType.setText("Channel/WirelessChannel");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(channelType, gridBagConstraints);

		jLabel4.setText("Network Interface Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel4, gridBagConstraints);

		nicType.setText("Phy/WirelessPhy");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(nicType, gridBagConstraints);

		jLabel5.setText("Interface Queue Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel5, gridBagConstraints);

		ifqType.setText("Queue/DropTail/PriQueue");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(ifqType, gridBagConstraints);

		jLabel8.setText("Antenna Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel8, gridBagConstraints);

		antennaModel.setText("Antenna/OmniAntenna");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennaModel, gridBagConstraints);

		jLabel12.setText("Ad-hoc Routing Protocol");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel12, gridBagConstraints);

		adhocProtocol.setText("AODV");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(adhocProtocol, gridBagConstraints);

		jLabel6.setText("Radio Propagation Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel6, gridBagConstraints);

		radioModel.setText("Propagation/TwoRayGround");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(radioModel, gridBagConstraints);

		jLabel7.setText("MAC Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel7, gridBagConstraints);

		macType.setText("Mac/802_11");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(macType, gridBagConstraints);

		jLabel10.setText("Max Packet in IFQ");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel10, gridBagConstraints);

		maxIFQ.setText("50");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(maxIFQ, gridBagConstraints);

		jLabel9.setText("Link Layer Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel9, gridBagConstraints);

		llType.setText("LL");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(llType, gridBagConstraints);

		jLabel22.setFont(new Font("Tahoma", 1, 11));
		jLabel22.setForeground(new Color(0, 51, 255));
		jLabel22.setText("Mobile Nodes starting positions");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 10, 0, 0);
		getContentPane().add(jLabel22, gridBagConstraints);

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		jTable1 = new JTable(mobnodestable);
		jScrollPane1.setViewportView(jTable1);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipadx = 330;
		gridBagConstraints.ipady = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jScrollPane1, gridBagConstraints);

		jLabel13.setText("Topology Boundary");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel13, gridBagConstraints);

		jLabel14.setText("x");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 8;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 10);
		getContentPane().add(jLabel14, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(xBoundary, gridBagConstraints);

		jLabel15.setText("y");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 8;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 10);
		getContentPane().add(jLabel15, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(yBoundary, gridBagConstraints);

		jLabel20.setText("Simulation Stop Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel20, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(stopTime, gridBagConstraints);

		jLabel11.setText("Mobile Nodes No");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel11, gridBagConstraints);

		mobileNodesNo.setEditable(false);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(mobileNodesNo, gridBagConstraints);

		macTraceCheck.setText("MAC Trace");
		macTraceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				macTraceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(macTraceCheck, gridBagConstraints);

		namTraceCheck.setText("NAM Trace");
		namTraceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				namTraceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(namTraceCheck, gridBagConstraints);

		agenTraceCheck.setText("Agent Trace");
		agenTraceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				agenTraceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(agenTraceCheck, gridBagConstraints);

		routerTraceCheck.setText("Router Trace");
		routerTraceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				routerTraceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(routerTraceCheck, gridBagConstraints);

		movementTraceCheck.setText("Movement Trace");
		movementTraceCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				movementTraceCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(movementTraceCheck, gridBagConstraints);

		jLabel25.setText("Set Nam Trace File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel25, gridBagConstraints);

		namPath.setEditable(false);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		getContentPane().add(namPath, gridBagConstraints);

		openNamButton.setText("...");
		openNamButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openNamButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(openNamButton, gridBagConstraints);

		jLabel21.setText("Set Trace Output File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel21, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		getContentPane().add(tracePath, gridBagConstraints);

		tracePathButton.setText("...");
		tracePathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				tracePathButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(tracePathButton, gridBagConstraints);

		onlyMovementCheck.setText("Only Generate Mobile Nodes Movement");
		onlyMovementCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				onlyMovementCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(onlyMovementCheck, gridBagConstraints);

		jLabel3.setFont(new Font("Tahoma", 1, 11));
		jLabel3.setForeground(new Color(0, 51, 255));
		jLabel3.setText("Connections");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 10, 0, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		jScrollPane2.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		jTable2 = new JTable(connstable);
		jScrollPane2.setViewportView(jTable2);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 15;
		gridBagConstraints.ipadx = 400;
		gridBagConstraints.ipady = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jScrollPane2, gridBagConstraints);

		addConnectionButton.setText("Add Connection");
		addConnectionButton.setMaximumSize(new Dimension(120, 23));
		addConnectionButton.setMinimumSize(new Dimension(120, 23));
		addConnectionButton.setPreferredSize(new Dimension(120, 23));
		addConnectionButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addConnectionButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 30;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 0);
		getContentPane().add(addConnectionButton, gridBagConstraints);

		removeConnectionButton.setText("Del Connection");
		removeConnectionButton.setMaximumSize(new Dimension(120, 23));
		removeConnectionButton.setMinimumSize(new Dimension(120, 23));
		removeConnectionButton.setPreferredSize(new Dimension(120, 23));
		removeConnectionButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeConnectionButtonActionPerformed(evt);
			}
		});
	
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 30;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(removeConnectionButton, gridBagConstraints);
		
		addTCP.setText("Set TCP");
		addTCP.setMaximumSize(new Dimension(80, 23));
		addTCP.setMinimumSize(new Dimension(80, 23));
		addTCP.setPreferredSize(new Dimension(80, 23));
		addTCP.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addTCPButtonActionPerformed(evt);
			}
		});
		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 30;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 0);
		getContentPane().add(addTCP, gridBagConstraints);

		
		addUDP.setText("Set UDP");
		addUDP.setMaximumSize(new Dimension(80, 23));
		addUDP.setMinimumSize(new Dimension(80, 23));
		addUDP.setPreferredSize(new Dimension(80, 23));
		addUDP.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addUDPButtonActionPerformed(evt);
			}
		});
		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 30;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(addUDP, gridBagConstraints);
		
		jLabel28.setFont(new Font("Tahoma", 1, 11));
		jLabel28.setForeground(new Color(0, 51, 255));
		jLabel28.setText("Agents Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridy = 24;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 80;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel28, gridBagConstraints);
		
		jLabel26.setFont(new Font("Tahoma", 1, 11));
		jLabel26.setText("UDP");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 25;
		gridBagConstraints.ipadx = 80;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel26, gridBagConstraints);

		jLabel31.setFont(new Font("Tahoma", 1, 11));
		jLabel31.setText("TCP");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 25;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel31, gridBagConstraints);

		jLabel40.setText("Packet size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel40, gridBagConstraints);

		UDPpacketsize.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(UDPpacketsize, gridBagConstraints);

		/*
		jLabel43.setText("Window Size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel43, gridBagConstraints);
		
		UDPwindowsize.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(UDPwindowsize, gridBagConstraints);
		*/
		
		jLabel41.setText("Start Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel41, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(UDPstarttime, gridBagConstraints);

		jLabel42.setText("Stop Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel42, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		//gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(UDPstoptime, gridBagConstraints);

		jLabel35.setText("Sending Rate");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel35, gridBagConstraints);

		UDPRate.setText("64Kb");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(UDPRate, gridBagConstraints);

		jLabel38.setText("Maximum Packets");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel38, gridBagConstraints);

		UDPMaxPacket.setText("2280000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(UDPMaxPacket, gridBagConstraints);

		UDPRandomCheck.setText("Introduce Random Noise");
		UDPRandomCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				UDPRandomCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipady = 3;		
		gridBagConstraints.insets = new Insets(-4, 0, 0, 0);
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;		
		getContentPane().add(UDPRandomCheck, gridBagConstraints);
		
		jLabel46.setText("Packet size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel46, gridBagConstraints);

		TCPpacketsize.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPpacketsize, gridBagConstraints);

		jLabel47.setText("Window Size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel47, gridBagConstraints);

		TCPwindowsize.setText("20");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPwindowsize, gridBagConstraints);

		jLabel50.setText("Start Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 6;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel50, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 7;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPstarttime, gridBagConstraints);

		jLabel49.setText("Stop Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel49, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPstoptime, gridBagConstraints);

		jLabel44.setText("Maximum Burst");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel44, gridBagConstraints);

		TCPmaxburst.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 27;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPmaxburst, gridBagConstraints);

		jLabel45.setText("Maximum cwnd");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel45, gridBagConstraints);

		TCPmaxcwnd.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(TCPmaxcwnd, gridBagConstraints);
	    
		FileMenu.setText("File");
		NewMenu.setText("New");
		NewMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				NewMenuActionPerformed(evt);
			}
		});

		FileMenu.add(NewMenu);

		importSUMOMenu.setText("Import MOVE Trace");
		importSUMOMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				try{
					importSUMOMenuActionPerformed(evt);
				}
				catch (Exception e){
				};
			}
		});

		FileMenu.add(importSUMOMenu);

		SaveMenu.setText("Save");
		SaveMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				SaveMenuActionPerformed(evt);
			}
		});

		FileMenu.add(SaveMenu);

		SaveAsMenu.setText("Save As");
		SaveAsMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				SaveAsMenuActionPerformed(evt);
			}
		});

		FileMenu.add(SaveAsMenu);

		QuitMenu.setText("Quit");
		QuitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				QuitMenuActionPerformed(evt);
			}
		});

		FileMenu.add(QuitMenu);

		jMenuBar1.add(FileMenu);

		HelpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		HelpMenu.add(quickHelpMenu);
		
		/*Quick help is disabled for now*/
		//jMenuBar1.add(HelpMenu);

		setJMenuBar(jMenuBar1);

		//java.awt.Dimension screenSize = java.awt.Toolkit.getDefaultToolkit().getScreenSize();
		//setBounds((screenSize.width-1000)/2, screenSize.height-1000, 950, 750);
//		setBounds(0, 0, 1000, 750);		
	}

	class NodesDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"Time","Node ID","Initial Position"};
		Vector vectorTableData = new Vector(10,3);

		public int getColumnCount() {
			return 3;
		}
		public Object getRow(int index){
			return vectorTableData.elementAt(index);
		}
		public int getRowCount() {
			return vectorTableData.size();
		}

		public String getColumnName(int col) {
			return columnNames[col];
		}

		public Object getValueAt(int row, int col) {
			try {
				Vector rowData = (Vector)vectorTableData.elementAt(row);
				if (rowData != null) {
					return rowData.elementAt(col);
				}
				return null;
			} catch (Exception e) {return null;}
			}

			public Class getColumnClass(int c) {
				return getValueAt(0, c).getClass();
			}

			public boolean isCellEditable(int row, int col) {
				if (col == 3)
				return true;
				return false;
			}

			public void setValueAt(Object value, int row, int col) {
				((Vector)vectorTableData.elementAt(row)).setElementAt(value, col);
				fireTableCellUpdated(row, col);
			}

			public void addRow(Vector someVector) {
				vectorTableData.addElement(someVector);
				fireTableRowsInserted(vectorTableData.size() - 1, vectorTableData.size() -1);
			}

			public void removeRow(int row) {
				vectorTableData.removeElementAt(row);
				fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
			}
		}

		class ConnectionsDynamicTableModel extends AbstractTableModel {
			private String[] columnNames = {"Source ID","Start time","Destination ID","End time","Transport Protocol"};
			Vector vectorTableData = new Vector(10,2);

			public int getColumnCount() {
				return 5;
			}

			public int getRowCount() {
				return vectorTableData.size();
			}

			public Object getRow(int index){
				return vectorTableData.elementAt(index);
			}

			public String getColumnName(int col) {
				return columnNames[col];
			}
			
			public Object getValueAt(int row, int col) {
				try {
					Vector rowData = (Vector)vectorTableData.elementAt(row);
					if (rowData != null) {
						return rowData.elementAt(col);
					}
					return null;
				} catch (Exception e) {return null;}
				}

				public Class getColumnClass(int c) {
					return getValueAt(0, c).getClass();
				}

				public boolean isCellEditable(int row, int col) {
					return true;
				}

				public void setValueAt(Object value, int row, int col) {
					((Vector)vectorTableData.elementAt(row)).setElementAt(value, col);
					fireTableCellUpdated(row, col);
				}

				public void addRow(Vector someVector) {
					vectorTableData.addElement(someVector);
					fireTableRowsInserted(vectorTableData.size() - 1, vectorTableData.size() -1);
				}

				public void removeRow(int row) {
					vectorTableData.removeElementAt(row);
					fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
				}
			}

			private void NewMenuActionPerformed(ActionEvent evt) {
				while (mobnodestable.getRowCount() != 0)
				mobnodestable.removeRow(mobnodestable.getRowCount()-1);
				while (connstable.getRowCount() != 0)
				connstable.removeRow(connstable.getRowCount()-1);
				filename = "untitled";
				hasimported = 0;

				channelType.setText("Channel/WirelessChannel");
				nicType.setText("Phy/WirelessPhy");
				ifqType.setText("Queue/DropTail/PriQueue");
				antennaModel.setText("Antenna/OmniAntenna");
				adhocProtocol.setText("AODV");
				radioModel.setText("Propagation/TwoRayGround");
				macType.setText("Mac/802_11");
				maxIFQ.setText("50");
				llType.setText("LL");

				mobileNodesNo.setText("");
				stopTime.setText("");
				tracePath.setText("");
				xBoundary.setText("");
				yBoundary.setText("");				

				/*set default for TCP, UDP, and SRM here*/
			}

			private void quickHelpMenuActionPerformed(ActionEvent evt) {
				JOptionPane.showMessageDialog(this, "Not yet available");
			}

			private void QuitMenuActionPerformed(ActionEvent evt) {
				this.dispose();
			}

			private void saveFile() {				
				int timeA = 0, timeB = 0;				

				for (int i = 0 ; i < connstable.getRowCount(); i++){
					NSNode tempN = (NSNode) ns.initialNode.get(i);
					ns.addConnections((Vector)connstable.getRow(i));
					
					timeA = Integer.parseInt(String.valueOf(connstable.getValueAt(i, 1)));
					timeB = Integer.parseInt(String.valueOf(connstable.getValueAt(i, 3)));
					ns.connectionTime(timeA, timeB, i, connstable.getRowCount());					
				}
				ns.setOption(channelType.getText(),radioModel.getText(),nicType.getText(),macType.getText(),ifqType.getText(),llType.getText(),antennaModel.getText(),maxIFQ.getText(),adhocProtocol.getText());
				ns.setGodParam(xBoundary.getText(),yBoundary.getText(),stopTime.getText(),flag1,flag2,flag3,flag4);

				//ns.setUDP(UDPstarttime.getText(),UDPstoptime.getText(),UDPwindowsize.getText(),UDPpacketsize.getText(),
				//UDPRate.getText(),UDPRate.getText(),UDPnoise,UDPMaxPacket.getText());
				ns.setUDP(UDPstarttime.getText(),UDPstoptime.getText(), "0",UDPpacketsize.getText(),
				UDPRate.getText(),UDPRate.getText(),UDPnoise,UDPMaxPacket.getText());

				ns.setTCP(TCPstarttime.getText(),TCPstoptime.getText(),TCPwindowsize.getText(),TCPpacketsize.getText(),
				TCPmaxburst.getText(),TCPmaxcwnd.getText());
				ns.setNamTrace(namflag,namPath.getText());

				ns.setOnlyMovement(onlyMovement);
				ns.write(filename,tracePath.getText());		
			}
			
			private void SaveMenuActionPerformed(ActionEvent evt) {
				if (hasimported == 1){
					if (filename == "untitled") {
						String extension = ".tcl";
						chooser.setFileFilter( new JaveFileFilter(extension) );
						int returnVal = chooser.showSaveDialog(this);
						if(returnVal == JFileChooser.APPROVE_OPTION) {
							filename=chooser.getSelectedFile().getPath();
							if ( filename.indexOf(extension) >= 0 )
								;
							else 
								filename=chooser.getSelectedFile().getPath() + extension;					
							saveFile();
						}
					}
					else
					{
						saveFile();
					}
				}
				else
				{
					JOptionPane.showMessageDialog(this, "Please import MOVE trace before");
				}
			}

		private void SaveAsMenuActionPerformed(ActionEvent evt) {
			if (hasimported == 1){
				String extension = ".tcl";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showSaveDialog(this);
				if(returnVal == JFileChooser.APPROVE_OPTION) {
					filename=chooser.getSelectedFile().getPath();
					if ( filename.indexOf(extension) >= 0 )
						;
					else 
						filename=chooser.getSelectedFile().getPath() + extension;				
					saveFile();					
				}
			}
			else
			{
				JOptionPane.showMessageDialog(this, "Please import MOVE trace before");
			}
		}

		private void importSUMOMenuActionPerformed(ActionEvent evt) throws Exception{
			JOptionPane.showMessageDialog(this, "Select MOVE Trace File");
			String extension = ".sumo.tr";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);			
			if(returnVal == JFileChooser.APPROVE_OPTION) {
						
			while (mobnodestable.getRowCount() != 0)
			mobnodestable.removeRow(mobnodestable.getRowCount()-1);
			while (connstable.getRowCount() != 0)
			connstable.removeRow(connstable.getRowCount()-1);
				
			sumoFile = chooser.getSelectedFile().getPath();
			JOptionPane.showMessageDialog(this, "Select MOVE .net.xml (map) File");
			String extension2 = ".net.xml";
			chooser.setFileFilter( new JaveFileFilter(extension2) );
			returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			netFile = chooser.getSelectedFile().getPath();

			ns = new buildNS(sumoFile, netFile, false);

			ns.build();
			
			for (int i = 0; i < ns.initialNode.size() ; i++){
				Vector newRow = new Vector();
				NSNode tempNode = (NSNode) ns.initialNode.get(i);
				newRow.addElement(tempNode.getTimestep());
				newRow.addElement(tempNode.getID());
				newRow.addElement(new String(tempNode.getX() + " " + tempNode.getY() + " 0.0"));				
				mobnodestable.addRow(newRow);				
				//System.out.println(tempNode.getTimestep());
			}
			mobileNodesNo.setText(ns.initialNode.size() + "");
			stopTime.setText(ns.returnSimTime());
			xBoundary.setText((((int)ns.returnXBoundary())+2) + "");
			yBoundary.setText((((int)ns.returnYBoundary())+2) + "");

			TCPstarttime.setText(ns.returnStartTime());
			TCPstoptime.setText(ns.returnSimTime());
			UDPstarttime.setText(ns.returnStartTime());
			UDPstoptime.setText(ns.returnSimTime());
						
			hasimported = 1;
			}
		}

		private void movementTraceCheckItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				flag4 = "OFF";//movementflag = 0;
			}
			else {
				flag4 = "ON";//movementflag = 1;
			}
		}
		private void macTraceCheckItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				flag3 = "OFF";//macflag = 0;
			}
			else {
				flag3 = "ON";//macflag = 1;
			}
		}
		private void routerTraceCheckItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				flag2 = "OFF";//routerflag = 0;
			}
			else {
				flag2 = "ON";//routerflag = 1;
			}
		}
		private void agenTraceCheckItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				flag1 = "OFF";//agentflag = 0;
			}
			else {
				flag1 = "ON";//agentflag = 1;
			}

		}

		private void tracePathButtonActionPerformed(ActionEvent evt) {
			String extension = ".tr";
			String openPath = "untitled";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION) {
				openPath=chooser.getSelectedFile().getPath();
			if ( openPath.indexOf(extension) >= 0 )
				;
			else 
				openPath=chooser.getSelectedFile().getPath() + extension;	
			tracePath.setText( openPath );
			}
		}

		private void addConnectionButtonActionPerformed(ActionEvent evt) {
			Vector newRow = new Vector();
			newRow.addElement(new String("0"));
			newRow.addElement(new String("0"));
			newRow.addElement(new String("0"));
			newRow.addElement(new String("0"));
			newRow.addElement(new String("0"));

			connstable.addRow(newRow);
		}
		private void removeConnectionButtonActionPerformed(ActionEvent evt) {
			if (connstable.getRowCount() != 0)
			connstable.removeRow(connstable.getRowCount()-1);			
		}
		
		private void addTCPButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<connstable.getRowCount(); i++)
				connstable.setValueAt( new String("tcp"), i, 4);
		}
		
		private void addUDPButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<connstable.getRowCount(); i++)
				connstable.setValueAt( new String("udp"), i, 4);
		}

		private void UDPRandomCheckItemStateChanged(java.awt.event.ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				UDPnoise = "NO";
			}
			else {
				UDPnoise = "YES";
			}
		}

		private void namTraceCheckItemStateChanged(java.awt.event.ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				namflag = false;
				namPath.setEditable(false);
			}
			else {
				namflag = true;
				namPath.setEditable(true);
			}
		}

		private void openNamButtonActionPerformed(ActionEvent evt) {
			String extension = ".nam";
			String openPath = "untitled";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION) {
				openPath=chooser.getSelectedFile().getPath();
			if ( openPath.indexOf(extension) >= 0 )
				;
			else 
				openPath=chooser.getSelectedFile().getPath() + extension;	
			namPath.setText( openPath );
			}
		}

		public static void main(String args[]) {
			EventQueue.invokeLater(new Runnable() {
				public void run() {
					new TCLEditor().setVisible(true);
				}
			});
		}

		private void onlyMovementCheckItemStateChanged(java.awt.event.ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
					onlyMovement = 0;
			}
			else {
					onlyMovement = 1;
			}
		}
	}

