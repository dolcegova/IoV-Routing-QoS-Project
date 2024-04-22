package com.move.qualnetPart;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.JaveFileFilter;
import com.move.ns2Part.NSNode;

import java.awt.*;
import java.io.*;
import java.awt.event.*;

public class qualnetEditor extends JFrame {

	private String filename = "untitled";
	private String sumofile = "untitled";
	private String netfile  = "untitled";
	PrintWriter apps;

	static JFileChooser chooser;
	private static int saveflag = 0;
	private static int hasimported = 0;

	private buildQualnet qnet;

	private NodesDynamicTableModel initialnodes = new NodesDynamicTableModel();
	
	/*General Options*/
	private JTextField experiment;
	private JTextField xboundary;
	private JTextField yboundary;
	private JTextField simtime;
	private JTextField numnodes;
	private JComboBox routingCombo;

	/*Radio or physical layer options*/
	private JComboBox physicalCombo;
	private JTextField datarate;
	private JTextField temperature;
	private JTextField noisefactor;
	private JTextField antennamodel;
	private JTextField antennagain;
	private JTextField antennaheight;
	private JTextField antennaefficiency;
	private JTextField antennamismatch;
	private JTextField antennacableloss;
	private JTextField antennaconnloss;

	/*Wireless options*/
	private JTextField frequency;
	private JTextField proplimit;
	private JComboBox pathlossCombo;
	private JComboBox shadowCombo;
	private JTextField shadowmean;
	private JComboBox fadingCombo;
	private JTextField gaussianfile;
	private JButton openGaussianButton;
	private JTextField maxvelocity;
	private JTextField riceankfactor;

	/*Mac802.11 options*/
	private JTextField longpacketlimit;
	private JTextField shortpacketlimit;
	private JTextField rtsthreshold;
	private JTextField propagationdelay;

	/*IP Network options*/
	private JTextField fragunit;
	private JTextField ipqueuesize;
	private JTextField loopback;

	/*Application options*/
	private ButtonGroup buttonGroup1;
	private JRadioButton cbroption;
	private JRadioButton ftpoption;
	private JRadioButton telnetoption;
	private JTextField itemsend;
	private JTextField itemsize;
	private JTextField interval;
	private JTextField starttime;
	private JTextField endtime;
	private JTextField sessduration;
	private JTextField precedence;

	public String appoption = "CBR";

	/*TCL options*/
	private JComboBox tcpCombo;
	private JCheckBox delayackschk;
	private JCheckBox delayshortpacketchk;
	private JCheckBox naglealgorithm;
	private JCheckBox pushchk;
	private JCheckBox keepalivechk;
	private JTextField sendbuffer;
	private JTextField receivebuffer;
	private JTextField maxsegmentsize;

	public String delayackflag = "NO";
	public String delayshortflag = "NO";
	public String nagleflag = "NO";
	public String pushflag = "NO";
	public String keepaliveflag = "NO";

	/*Tables*/
	private JTable stattable;
	private JTable mobilenodestable;
	private JScrollPane jScrollPane1;
	private JScrollPane jScrollPane3;

	/*Menu Items*/
	private JMenuBar jMenuBar1;
	private JMenu FileMenu;
	private JMenuItem openTCLMenu;
	private JMenu HelpMenu;
	private JMenuItem NewMenu;
	private JMenuItem QuitMenu;
	private JMenuItem SaveAsMenu;
	private JMenuItem SaveMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem importSUMOMenu;

	/*Misc labels*/
	private JLabel title;
	private JLabel label1;
	private JLabel label10;
	private JLabel label11;
	private JLabel label12;
	private JLabel label13;
	private JLabel label14;
	private JLabel label15;
	private JLabel label16;
	private JLabel label17;
	private JLabel label18;
	private JLabel label19;
	private JLabel label2;
	private JLabel label20;
	private JLabel label21;
	private JLabel label22;
	private JLabel label23;
	private JLabel label24;
	private JLabel label25;
	private JLabel label26;
	private JLabel label27;
	private JLabel label28;
	private JLabel label29;
	private JLabel label3;
	private JLabel label30;
	private JLabel label4;
	private JLabel label5;
	private JLabel label6;
	private JLabel label7;
	private JLabel label8;
	private JLabel label9;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel12;
	private JLabel jLabel16;
	private JLabel jLabel17;
	private JLabel jLabel18;
	private JLabel jLabel19;
	private JLabel jLabel30;
	private JLabel jLabel31;
	private JLabel jLabel32;
	private JLabel jLabel33;
	private JLabel jLabel34;
	private JLabel jLabel35;
	private JLabel jLabel36;
	private JLabel jLabel37;
	private JLabel jLabel38;
	private JLabel jLabel39;
	private JLabel jLabel40;
	private JLabel jLabel41;
	private JLabel jLabel42;
	private JLabel jLabel43;
	private JLabel jLabel44;
	private JLabel jLabel45;
	private JLabel jLabel46;
	private JLabel jLabel47;
	private JLabel jLabel48;
	private JLabel jLabel57;
	private JLabel jLabel58;
	private JLabel jLabel59;
	private JLabel jLabel60;
	private JLabel jLabel61;
	private JLabel jLabel70;
	private JLabel jLabel71;
	private JLabel jLabel72;
	private JLabel jLabel79;
	private JLabel jLabel80;
	private JLabel jLabel81;
	private JLabel jLabel83;
	private JLabel jLabel86;
	private JLabel jLabel88;
	private JLabel jLabel89;
	private JLabel jLabel9;
	private JLabel jLabel90;

	public qualnetEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		buttonGroup1 = new ButtonGroup();
		title = new JLabel();
		label1 = new JLabel();
		label2 = new JLabel();
		label3 = new JLabel();
		experiment = new JTextField();
		label4 = new JLabel();
		xboundary = new JTextField();
		label5 = new JLabel();
		yboundary = new JTextField();
		label6 = new JLabel();
		label8 = new JLabel();
		label9 = new JLabel();
		simtime = new JTextField();
		numnodes = new JTextField();
		label7 = new JLabel();
		routingCombo = new JComboBox();
		label10 = new JLabel();
		label11 = new JLabel();
		label12 = new JLabel();
		physicalCombo = new JComboBox();
		datarate = new JTextField();
		label13 = new JLabel();
		label14 = new JLabel();
		temperature = new JTextField();
		label15 = new JLabel();
		label16 = new JLabel();
		noisefactor = new JTextField();
		label17 = new JLabel();
		antennamodel = new JTextField();
		label18 = new JLabel();
		antennagain = new JTextField();
		label24 = new JLabel();
		label19 = new JLabel();
		antennaheight = new JTextField();
		label25 = new JLabel();
		label20 = new JLabel();
		antennaefficiency = new JTextField();
		label26 = new JLabel();
		label21 = new JLabel();
		antennamismatch = new JTextField();
		label27 = new JLabel();
		label22 = new JLabel();
		antennacableloss = new JTextField();
		label28 = new JLabel();
		label23 = new JLabel();
		antennaconnloss = new JTextField();
		label29 = new JLabel();
		label30 = new JLabel();
		jLabel9 = new JLabel();
		jLabel10 = new JLabel();
		frequency = new JTextField();
		jLabel70 = new JLabel();
		jLabel11 = new JLabel();
		proplimit = new JTextField();
		jLabel71 = new JLabel();
		jLabel12 = new JLabel();
		pathlossCombo = new JComboBox();
		jLabel17 = new JLabel();
		shadowCombo = new JComboBox();
		jLabel16 = new JLabel();
		shadowmean = new JTextField();
		jLabel36 = new JLabel();
		fadingCombo = new JComboBox();
		jLabel19 = new JLabel();
		maxvelocity = new JTextField();
		jLabel18 = new JLabel();
		gaussianfile = new JTextField();
		openGaussianButton = new JButton();
		jLabel37 = new JLabel();
		riceankfactor = new JTextField();
		jLabel30 = new JLabel();
		jLabel32 = new JLabel();
		shortpacketlimit = new JTextField();
		jLabel33 = new JLabel();
		longpacketlimit = new JTextField();
		jLabel34 = new JLabel();
		rtsthreshold = new JTextField();
		jLabel31 = new JLabel();
		propagationdelay = new JTextField();
		jLabel35 = new JLabel();
		jLabel39 = new JLabel();
		fragunit = new JTextField();
		jLabel42 = new JLabel();
		ipqueuesize = new JTextField();
		jLabel44 = new JLabel();
		loopback = new JTextField();
		jLabel60 = new JLabel();
		jLabel61 = new JLabel();
		cbroption = new JRadioButton();
		ftpoption = new JRadioButton();
		telnetoption = new JRadioButton();
		jLabel40 = new JLabel();
		itemsend = new JTextField();
		jLabel41 = new JLabel();
		itemsize = new JTextField();
		jLabel43 = new JLabel();
		jLabel79 = new JLabel();
		interval = new JTextField();
		jLabel83 = new JLabel();
		jLabel45 = new JLabel();
		starttime = new JTextField();
		jLabel80 = new JLabel();
		jLabel46 = new JLabel();
		endtime = new JTextField();
		jLabel81 = new JLabel();
		jLabel86 = new JLabel();
		sessduration = new JTextField();
		jLabel88 = new JLabel();
		jLabel72 = new JLabel();
		precedence = new JTextField();
		jLabel48 = new JLabel();
		jLabel47 = new JLabel();
		tcpCombo = new JComboBox();
		delayackschk = new JCheckBox();
		delayshortpacketchk = new JCheckBox();
		naglealgorithm = new JCheckBox();
		pushchk = new JCheckBox();
		keepalivechk = new JCheckBox();
		jLabel57 = new JLabel();
		sendbuffer = new JTextField();
		jLabel90 = new JLabel();
		jLabel58 = new JLabel();
		receivebuffer = new JTextField();
		jLabel89 = new JLabel();
		jLabel59 = new JLabel();
		maxsegmentsize = new JTextField();
		jLabel38 = new JLabel();
		jScrollPane1 = new JScrollPane();
		mobilenodestable = new JTable();
		jScrollPane3 = new JScrollPane();
		stattable = new JTable();
		jMenuBar1 = new JMenuBar();
		FileMenu = new JMenu();
		NewMenu = new JMenuItem();
		importSUMOMenu = new JMenuItem();
		openTCLMenu = new JMenuItem();
		SaveMenu = new JMenuItem();
		SaveAsMenu = new JMenuItem();
		QuitMenu = new JMenuItem();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();

		getContentPane().setLayout(new GridBagLayout());

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("Qualnet Simulation Generator");
		if(chooser == null) {
			chooser = new JFileChooser();
		}

		title.setFont(new Font("Arial", 1, 24));
		title.setForeground(new Color(255, 0, 51));
		title.setHorizontalAlignment(SwingConstants.LEFT);
		title.setText("Qualnet Simulation Generator (4.0)");

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 10;
		gridBagConstraints.ipadx = 30;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 0, 10, 0);
		getContentPane().add(title, gridBagConstraints);

		label1.setFont(new Font("Tahoma", 1, 11));
		label1.setForeground(new Color(0, 51, 255));
		label1.setText("General Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label1, gridBagConstraints);

		label2.setText("Experiment Name");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label2, gridBagConstraints);

		label3.setText("Topology Boundary");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label3, gridBagConstraints);

		experiment.setText("Experiment");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 70;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(experiment, gridBagConstraints);

		label4.setText("x");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label4, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(xboundary, gridBagConstraints);

		label5.setText("y");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(label5, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(yboundary, gridBagConstraints);

		label6.setText("Simulation Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label6, gridBagConstraints);

		label8.setText("Number of Nodes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label8, gridBagConstraints);

		label9.setText("Routing Protocol");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label9, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(simtime, gridBagConstraints);

		numnodes.setEditable(false);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(numnodes, gridBagConstraints);

		label7.setText("second");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label7, gridBagConstraints);

		routingCombo.setModel(new DefaultComboBoxModel(new String[] { "AODV", "BELLMANFORD", "RIP" }));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(routingCombo, gridBagConstraints);

		label10.setFont(new Font("Tahoma", 1, 11));
		label10.setForeground(new Color(0, 51, 255));
		label10.setText("Radio/Physical Layer Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label10, gridBagConstraints);

		label11.setText("Physical Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label11, gridBagConstraints);

		label12.setText("Data rate");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label12, gridBagConstraints);

		physicalCombo.setModel(new DefaultComboBoxModel(new String[] { "802.11a", "802.11b" }));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(physicalCombo, gridBagConstraints);

		datarate.setText("6000000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 60;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(datarate, gridBagConstraints);

		label13.setText("Hz");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label13, gridBagConstraints);

		label14.setText("Temperature");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label14, gridBagConstraints);

		temperature.setText("290");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(temperature, gridBagConstraints);

		label15.setText("K");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label15, gridBagConstraints);

		label16.setText("Noise Factor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label16, gridBagConstraints);

		noisefactor.setText("7.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(noisefactor, gridBagConstraints);

		label17.setText("Antenna Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label17, gridBagConstraints);

		antennamodel.setText("OMNIDIRECTIONAL");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 120;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennamodel, gridBagConstraints);

		label18.setText("Antenna Gain");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label18, gridBagConstraints);

		antennagain.setText("0.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennagain, gridBagConstraints);

		label24.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label24, gridBagConstraints);

		label19.setText("Antenna Height");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label19, gridBagConstraints);

		antennaheight.setText("1.5");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennaheight, gridBagConstraints);

		label25.setText("m");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label25, gridBagConstraints);

		label20.setText("Antenna Efficiency");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label20, gridBagConstraints);

		antennaefficiency.setText("0.8");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennaefficiency, gridBagConstraints);

		label26.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label26, gridBagConstraints);

		label21.setText("Antenna Mismatch Loss");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label21, gridBagConstraints);

		antennamismatch.setText("0.3");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennamismatch, gridBagConstraints);

		label27.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label27, gridBagConstraints);

		label22.setText("Antenna Cable Loss");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label22, gridBagConstraints);

		antennacableloss.setText("0.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennacableloss, gridBagConstraints);

		label28.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label28, gridBagConstraints);

		label23.setText("Antenna Connection Loss");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label23, gridBagConstraints);

		antennaconnloss.setText("0.2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(antennaconnloss, gridBagConstraints);

		label29.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(label29, gridBagConstraints);

		label30.setFont(new Font("Tahoma", 1, 11));
		label30.setForeground(new Color(0, 51, 255));
		label30.setText("Mobile Nodes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 22;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(label30, gridBagConstraints);

		jLabel9.setFont(new Font("Tahoma", 1, 11));
		jLabel9.setForeground(new Color(0, 51, 255));
		jLabel9.setText("Wireless Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel9, gridBagConstraints);

		jLabel10.setText("Frequency");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel10, gridBagConstraints);

		frequency.setText("2400000000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(frequency, gridBagConstraints);

		jLabel70.setText("Hz");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 12;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel70, gridBagConstraints);

		jLabel11.setText("Propagation Limit");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel11, gridBagConstraints);

		proplimit.setText("-111.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(proplimit, gridBagConstraints);

		jLabel71.setText("dB");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 11;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel71, gridBagConstraints);

		jLabel12.setText("Propagation Pathloss Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel12, gridBagConstraints);

		pathlossCombo.setModel(new DefaultComboBoxModel(new String[] { "TWO-RAY", "FREE-SPACE" }));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(pathlossCombo, gridBagConstraints);

		jLabel17.setText("Propagation Shadowing Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel17, gridBagConstraints);

		shadowCombo.setModel(new DefaultComboBoxModel(new String[] { "CONSTANT", "NONE", "LOGNORMAL" }));
		shadowCombo.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				shadowComboActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(shadowCombo, gridBagConstraints);

		jLabel16.setText("Propagation Shadowing Mean");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel16, gridBagConstraints);

		shadowmean.setText("4.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(shadowmean, gridBagConstraints);

		jLabel36.setText("Propagation Fading Model");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel36, gridBagConstraints);

		fadingCombo.setModel(new DefaultComboBoxModel(new String[] {"RAYLEIGH", "NONE","RICEAN"}));
		fadingCombo.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				fadingComboActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 10;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(fadingCombo, gridBagConstraints);

		jLabel19.setText("Propagation Fading Max Velocity");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel19, gridBagConstraints);

		maxvelocity.setText("10.0");
		maxvelocity.setEditable(true);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(maxvelocity, gridBagConstraints);

		jLabel18.setText("Propagation Gaussian Components File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 10);
		getContentPane().add(jLabel18, gridBagConstraints);

		gaussianfile.setText("default.fading");
		gaussianfile.setEditable(true);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.anchor = GridBagConstraints.EAST;
		getContentPane().add(gaussianfile, gridBagConstraints);

		openGaussianButton.setText("...");
		openGaussianButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openGaussianButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 0, 10);
		getContentPane().add(openGaussianButton, gridBagConstraints);

		jLabel37.setText("Propagation Ricean K Factor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel37, gridBagConstraints);


		riceankfactor.setText("0.0");
		riceankfactor.setEditable(false);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(riceankfactor, gridBagConstraints);

		jLabel30.setFont(new Font("Tahoma", 1, 11));
		jLabel30.setForeground(new Color(0, 51, 255));
		jLabel30.setText("MAC802.11 Protocol Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel30, gridBagConstraints);

		jLabel32.setText("Short Packet Transmit Limit");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel32, gridBagConstraints);

		shortpacketlimit.setText("7");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(shortpacketlimit, gridBagConstraints);

		jLabel33.setText("Long Packet Transmit Limit");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel33, gridBagConstraints);

		longpacketlimit.setText("4");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(longpacketlimit, gridBagConstraints);

		jLabel34.setText("RTS Threshold");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel34, gridBagConstraints);

		rtsthreshold.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(rtsthreshold, gridBagConstraints);

		jLabel31.setText("Propagation Delay");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel31, gridBagConstraints);

		propagationdelay.setText("1US");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(propagationdelay, gridBagConstraints);

		jLabel35.setFont(new Font("Tahoma", 1, 11));
		jLabel35.setForeground(new Color(0, 51, 255));
		jLabel35.setText("IP Network Protocol Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel35, gridBagConstraints);

		jLabel39.setText("Fragmentation Unit");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel39, gridBagConstraints);

		fragunit.setText("2048");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(fragunit, gridBagConstraints);

		jLabel42.setText("IP Priority Queue Size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 19;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel42, gridBagConstraints);

		ipqueuesize.setText("50000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 19;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(ipqueuesize, gridBagConstraints);

		jLabel44.setText("IP Loopback Address");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel44, gridBagConstraints);

		loopback.setText("127.0.0.1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 10;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(loopback, gridBagConstraints);

		jLabel60.setFont(new Font("Tahoma", 1, 11));
		jLabel60.setForeground(new Color(0, 51, 255));
		jLabel60.setText("Trace and Statistics Settings");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 22;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(jLabel60, gridBagConstraints);

		jLabel61.setFont(new Font("Tahoma", 1, 11));
		jLabel61.setForeground(new Color(0, 51, 255));
		jLabel61.setText("Applications Layer Options");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel61, gridBagConstraints);

		buttonGroup1.add(cbroption);
		cbroption.setSelected(true);
		cbroption.setText("CBR");
		cbroption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cbroptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(cbroption, gridBagConstraints);

		buttonGroup1.add(ftpoption);
		ftpoption.setText("FTP");
		ftpoption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				ftpoptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(ftpoption, gridBagConstraints);

		buttonGroup1.add(telnetoption);
		telnetoption.setText("TELNET");
		telnetoption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				telnetoptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(telnetoption, gridBagConstraints);

		jLabel40.setText("Item to Send");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel40, gridBagConstraints);

		itemsend.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(itemsend, gridBagConstraints);

		jLabel41.setText("Item Size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel41, gridBagConstraints);

		itemsize.setText("512");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(itemsize, gridBagConstraints);

		jLabel43.setText("bytes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel43, gridBagConstraints);

		jLabel79.setText("Interval");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel79, gridBagConstraints);

		interval.setEditable(true);
		interval.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(interval, gridBagConstraints);

		jLabel83.setText("second");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel83, gridBagConstraints);

		jLabel45.setText("Start Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel45, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(starttime, gridBagConstraints);

		jLabel80.setText("second");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel80, gridBagConstraints);

		jLabel46.setText("End Time");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel46, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(endtime, gridBagConstraints);

		jLabel81.setText("second");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel81, gridBagConstraints);

		jLabel86.setText("Session Duration");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel86, gridBagConstraints);

		sessduration.setEditable(false);
		sessduration.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(sessduration, gridBagConstraints);

		jLabel88.setText("second");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel88, gridBagConstraints);

		jLabel72.setText("Precedence Bits");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel72, gridBagConstraints);

		precedence.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(precedence, gridBagConstraints);

		jLabel48.setFont(new Font("Tahoma", 1, 11));
		jLabel48.setForeground(new Color(0, 51, 255));
		jLabel48.setText("TCP Settings");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel48, gridBagConstraints);

		jLabel47.setText("Type");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel47, gridBagConstraints);

		tcpCombo.setModel(new DefaultComboBoxModel(new String[] { "LITE", "TAHOE", "RENO", "NEWRENO", "SACK" }));
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 13;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(tcpCombo, gridBagConstraints);

		delayackschk.setText("Delay ACKS");
		delayackschk.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				delayackschkItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(delayackschk, gridBagConstraints);

		delayshortpacketchk.setText("Delay Short Packet ACKS");
		delayshortpacketchk.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				delayshortpacketchkItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 15;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(delayshortpacketchk, gridBagConstraints);

		naglealgorithm.setText("Use Nagle Algorithm");
		naglealgorithm.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				naglealgorithmItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 16;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(naglealgorithm, gridBagConstraints);

		pushchk.setText("Use Push");
		pushchk.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				pushchkItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 17;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(pushchk, gridBagConstraints);

		keepalivechk.setText("Use Keepalive Probes");
		keepalivechk.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				keepalivechkItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 18;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(keepalivechk, gridBagConstraints);

		jLabel57.setText("Send buffer");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 19;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel57, gridBagConstraints);

		sendbuffer.setText("16384");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 19;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(sendbuffer, gridBagConstraints);

		jLabel90.setText("bytes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 19;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel90, gridBagConstraints);

		jLabel58.setText("Receive buffer");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel58, gridBagConstraints);

		receivebuffer.setText("16384");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(receivebuffer, gridBagConstraints);

		jLabel89.setText("bytes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 20;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel89, gridBagConstraints);

		jLabel59.setText("Maximum Segment Size");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 13;
		gridBagConstraints.gridy = 21;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel59, gridBagConstraints);

		maxsegmentsize.setText("512");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 14;
		gridBagConstraints.gridy = 21;
		gridBagConstraints.ipadx = 40;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(maxsegmentsize, gridBagConstraints);

		jLabel38.setText("bytes");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 15;
		gridBagConstraints.gridy = 21;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 10);
		getContentPane().add(jLabel38, gridBagConstraints);

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		mobilenodestable = new JTable(initialnodes);
		jScrollPane1.setViewportView(mobilenodestable);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 23;
		gridBagConstraints.gridwidth = 8;
		gridBagConstraints.gridheight = 20;
		gridBagConstraints.ipadx = 250;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 10, 10);
		getContentPane().add(jScrollPane1, gridBagConstraints);

		stattable.setModel(new DefaultTableModel(
		new Object [][] {
			{"Application Stats", Boolean.FALSE},
			{"TCP Stats", Boolean.FALSE},
			{"Routing Stats", Boolean.FALSE},
			{"IGMP Stats", Boolean.FALSE},
			{"Exterior Gateway Protocol Stats", Boolean.FALSE},
			{"Network Layer Stats", Boolean.FALSE},
			{"Queue Stats", Boolean.FALSE},
			{"MAC Layer Stats", Boolean.FALSE},
			{"Phy Layer Stats", Boolean.FALSE},
			{"Mobility Stats", Boolean.FALSE},
			{"RSVP Stats", Boolean.FALSE},
			{"Diffserv Edge Router Stats", Boolean.FALSE},
			{"Access List Stats", Boolean.FALSE}
		},
		new String [] {
			"Type", "Settings"
		}
		) {
			Class[] types = new Class [] {
				java.lang.Object.class, java.lang.Boolean.class
			};

			public Class getColumnClass(int columnIndex) {
				return types [columnIndex];
			}
		});
		jScrollPane3.setViewportView(stattable);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 9;
		gridBagConstraints.gridy = 23;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.gridheight = 20;
		gridBagConstraints.ipadx = 250;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 0, 10, 0);
		getContentPane().add(jScrollPane3, gridBagConstraints);

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
				catch (Exception e){}
			}
		});

		FileMenu.add(importSUMOMenu);

		openTCLMenu.setText("Open");
		openTCLMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openTCLMenuActionPerformed(evt);
			}
		});

		//FileMenu.add(openTCLMenu);

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
		
		FileMenu.setText("File");
		NewMenu.setText("New");
		NewMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				NewMenuActionPerformed(evt);
			}
		});	

		FileMenu.add(QuitMenu);
		jMenuBar1.add(FileMenu);
		HelpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		HelpMenu.add(quickHelpMenu);
		/*Quick help is disabled for now*/
		//jMenuBar1.add(HelpMenu);
		setJMenuBar(jMenuBar1);
		setBounds(0,0,1000,750);
		setVisible(true);
	}

	class NodesDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"ID","Initial Positions"};
		Vector vectorTableData = new Vector(10,2);
		public int getColumnCount() {
			return 2;
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
				} catch (Exception e) {
					return null;
				}
			}
			public Class getColumnClass(int c) {
				return getValueAt(0, c).getClass();
			}
			public boolean isCellEditable(int row, int col) {
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
			public void clearTable() {
				int i = getRowCount();
				while (i > 0){
					removeRow(i-1);
					i--;
				}
			}
		}

		private void shadowComboActionPerformed(ActionEvent evt) {
			if (shadowCombo.getSelectedIndex() == 1){
				shadowmean.setEditable(false);
			}
			else shadowmean.setEditable(true);
		}
		private void fadingComboActionPerformed(ActionEvent evt) {
			int x = fadingCombo.getSelectedIndex();
			if (x == 2){
				gaussianfile.setEditable(true);
				maxvelocity.setEditable(true);
				riceankfactor.setEditable(true);
			}
			else if (x == 1){
				gaussianfile.setEditable(false);
				maxvelocity.setEditable(false);
				riceankfactor.setEditable(false);
			}
			else{
				gaussianfile.setEditable(true);
				maxvelocity.setEditable(true);
				riceankfactor.setEditable(false);
			}
		}
		private void openGaussianButtonActionPerformed(ActionEvent evt) {
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			gaussianfile.setText(chooser.getSelectedFile().getPath());
		}


		private void cbroptionActionPerformed(ActionEvent evt) {
			itemsend.setEditable(true);
			itemsize.setEditable(true);
			interval.setEditable(true);
			starttime.setEditable(true);
			endtime.setEditable(true);
			sessduration.setEditable(false);
			precedence.setEditable(true);
			appoption = "CBR";
		}
		private void ftpoptionActionPerformed(ActionEvent evt) {
			itemsend.setEditable(true);
			itemsize.setEditable(true);
			interval.setEditable(false);
			starttime.setEditable(true);
			endtime.setEditable(true);
			sessduration.setEditable(false);
			precedence.setEditable(true);
			appoption = "FTP";
		}
		private void telnetoptionActionPerformed(ActionEvent evt) {
			itemsend.setEditable(false);
			itemsize.setEditable(false);
			interval.setEditable(false);
			starttime.setEditable(true);
			endtime.setEditable(false);
			sessduration.setEditable(true);
			precedence.setEditable(false);
			appoption = "TELNET";
		}


		private void keepalivechkItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				keepaliveflag = "NO";
			}
			else {
				keepaliveflag = "YES";
			}
		}

		private void pushchkItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				pushflag = "NO";
			}
			else {
				pushflag = "YES";
			}
		}

		private void naglealgorithmItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				nagleflag = "NO";
			}
			else {
				nagleflag = "YES";
			}
		}

		private void delayshortpacketchkItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				delayshortflag = "NO";
			}
			else {
				delayshortflag = "YES";
			}
		}

		private void delayackschkItemStateChanged(ItemEvent evt) {
			if (evt.getStateChange() == ItemEvent.DESELECTED) {
				delayackflag = "NO";
			}
			else {
				delayackflag = "YES";
			}
		}

		private void QuitMenuActionPerformed(ActionEvent evt) {
			this.dispose();
		}

		private void saveFile() {
			qnet.setGeneral(experiment.getText(),xboundary.getText(),yboundary.getText(),
			simtime.getText(),numnodes.getText(),(String)routingCombo.getSelectedItem());
			qnet.setPhysical((String)physicalCombo.getSelectedItem(),datarate.getText(),temperature.getText(),
			noisefactor.getText(),antennamodel.getText(),antennagain.getText(),antennaheight.getText(),
			antennaefficiency.getText(),antennamismatch.getText(),antennacableloss.getText(),
			antennaconnloss.getText());

			qnet.setWireless(frequency.getText(),proplimit.getText(),(String)pathlossCombo.getSelectedItem(),
			(String)shadowCombo.getSelectedItem(),shadowmean.getText(),(String)fadingCombo.getSelectedItem(),
			gaussianfile.getText(),maxvelocity.getText(),riceankfactor.getText());

			qnet.setMac(longpacketlimit.getText(),shortpacketlimit.getText(),rtsthreshold.getText(),propagationdelay.getText());

			qnet.setIP(fragunit.getText(),ipqueuesize.getText(),loopback.getText());

			qnet.setTCL((String)tcpCombo.getSelectedItem(),delayackflag,delayshortflag,nagleflag,pushflag,keepaliveflag,
			sendbuffer.getText(),receivebuffer.getText(),maxsegmentsize.getText());

			String temp = "";
			for (int i = 0; i < 13; i++){
				if ((Boolean)stattable.getValueAt(i,1) == Boolean.TRUE)
				temp = "YES";
				else
				temp = "NO";

				qnet.addStat(temp);
			}


			qnet.write(filename);

			try{
				apps = new PrintWriter(new  BufferedWriter(new FileWriter(filename + ".app")));
			}
			catch(IOException e){};
			if (appoption.equals("CBR"))
			apps.print(appoption + " 1 " + numnodes.getText() + " " + itemsend.getText() + " " + itemsize.getText() + " " + interval.getText() + "S " + starttime.getText() + "S " + endtime.getText() + "S PRECEDENCE " + precedence.getText() +"\n");
			else if (appoption.equals("FTP"))
			apps.print(appoption + " 1 " + numnodes.getText() + " " + itemsend.getText() + " " + itemsize.getText() + " " + starttime.getText() + "S " + endtime.getText() + "S PRECEDENCE " + precedence.getText() +"\n");
			else
			apps.print(appoption + " 1 " + numnodes.getText() + " " + sessduration.getText() + "S " + starttime.getText() + "S\n");
			apps.close();
		}
		
		private void SaveMenuActionPerformed(ActionEvent evt) {
			if (hasimported == 1){
				if (filename == "untitled"){
					String extension = ".config";
					chooser.setFileFilter( new JaveFileFilter(extension) );
					int returnVal = chooser.showSaveDialog(this);
					if (returnVal == JFileChooser.APPROVE_OPTION){
						filename=chooser.getSelectedFile().getPath();
						if ( filename.indexOf(extension) >= 0 )
							;
						else 
							filename=chooser.getSelectedFile().getPath() + extension;				
						saveFile();
					}
				}
				else {
					saveFile();
					
				}			
			}
		}
		
		private void SaveAsMenuActionPerformed(ActionEvent evt) {
			if (hasimported == 1){
				String extension = ".config";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showSaveDialog(this);
				if (returnVal == JFileChooser.APPROVE_OPTION){
					filename=chooser.getSelectedFile().getPath();
					if ( filename.indexOf(extension) >= 0 )
						;
					else 
						filename=chooser.getSelectedFile().getPath() + extension;				
					saveFile();
				}
			}
		}

		private void openTCLMenuActionPerformed(ActionEvent evt) {
			JOptionPane.showMessageDialog(this, "Not yet available");
		}

		private void importSUMOMenuActionPerformed(ActionEvent evt) throws Exception{
			initialnodes.clearTable();

			JOptionPane.showMessageDialog(this, "Select MOVE Trace File");
			String extension = ".sumo.tr";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if (returnVal == JFileChooser.APPROVE_OPTION)
			sumofile = chooser.getSelectedFile().getPath();

			JOptionPane.showMessageDialog(this, "Select MOVE .net.xml (Map) File");
			String extension2 = ".net.xml";
			chooser.setFileFilter( new JaveFileFilter(extension2) );
			returnVal = chooser.showOpenDialog(this);
			if (returnVal == JFileChooser.APPROVE_OPTION)
			netfile = chooser.getSelectedFile().getPath();

			qnet = new buildQualnet(sumofile,netfile);
			qnet.build();

			for (int i = 0; i < qnet.initialNode.size(); i++){
				Vector newRow = new Vector();
				NSNode tempNode = (NSNode) qnet.initialNode.get(i);
				newRow.addElement(tempNode.getID());
				newRow.addElement(new String(tempNode.getX() + " " + tempNode.getY() + " 0.0"));
				initialnodes.addRow(newRow);

			}
			numnodes.setText(qnet.initialNode.size() + "");
			xboundary.setText(qnet.returnXBoundary() + "");
			yboundary.setText(qnet.returnYBoundary() + "");
			simtime.setText(qnet.returnSimTime());
			starttime.setText(qnet.returnStartTime());
			endtime.setText(qnet.returnSimTime());
			hasimported = 1;
		}

		private void NewMenuActionPerformed(ActionEvent evt) {
			initialnodes.clearTable();
			filename = "untitled";
			hasimported = 0;

			/*Initialize all text boxes here*/
		}

		public static void main(String args[]) {
			new qualnetEditor();
		}
}
