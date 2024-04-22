package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;
import java.io.*;

public class autoRoutes extends JFrame {
	private String filename = "untitled";
	private int osenv;
	static JFileChooser chooser;
	
	private String workDirFile = "workPath.txt";	

	private static int option = 1; 
	private static int duaoption = 0;

	private JTextArea statusText;
	private JButton OKButton;
	private JButton cancelButton;
	private ButtonGroup buttonGroup1;
	private ButtonGroup buttonGroup2;

	private JRadioButton duaOption;
	private JRadioButton tripDefOption;
	private JRadioButton randompersecOption;
	private JRadioButton flowDefOption;
	private JRadioButton bothOption;
	private JRadioButton jtrOption;

	private JTextField randompersecond;
	private JTextField flowduaPath;
	private JTextField tripsPath;
	private JTextField turnPath;
	private JTextField flowjtrPath;

	private JTextField end;
	private JTextField begin;

	private JTextField netPath;
	private JTextField outputPath;

	private JMenuBar jMenuBar;
	private JMenu helpMenu;
	private JMenuItem quickHelpMenu;
	private JPanel jPanel1;

	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JLabel jLabel5;
	private JLabel jLabel6;
	private JLabel jLabel7;
	private JLabel jLabel8;

	private JButton openNet;
	private JButton openOutput;
	private JButton openTrips;
	private JButton openTurn;
	private JButton openduaFlow;
	private JButton openjtrFlow;

	public autoRoutes(String sumoPath, int os) {
		osenv = os;
		filename = sumoPath;
		initComponents(sumoPath, os);
	}

	private void initComponents(String sumoPath, int os) {
		GridBagConstraints gridBagConstraints;

		buttonGroup1 = new ButtonGroup();
		buttonGroup2 = new ButtonGroup();
		jPanel1 = new JPanel();
		jLabel1 = new JLabel();
		jtrOption = new JRadioButton();
		duaOption = new JRadioButton();
		jLabel2 = new JLabel();
		jLabel3 = new JLabel();
		jLabel4 = new JLabel();
		outputPath = new JTextField();
		begin = new JTextField();
		end = new JTextField();
		netPath = new JTextField();
		jLabel5 = new JLabel();
		flowjtrPath = new JTextField();
		turnPath = new JTextField();
		randompersecond = new JTextField();
		tripsPath = new JTextField();
		flowduaPath = new JTextField();
		OKButton = new JButton();
		cancelButton = new JButton();
		jLabel6 = new JLabel();
		openjtrFlow = new JButton();
		openTurn = new JButton();
		openNet = new JButton();
		openOutput = new JButton();
		openTrips = new JButton();
		openduaFlow = new JButton();
		jLabel7 = new JLabel();
		jLabel8 = new JLabel();
		randompersecOption = new JRadioButton();
		flowDefOption = new JRadioButton();
		tripDefOption = new JRadioButton();
		bothOption = new JRadioButton();
		statusText = new JTextArea();
		jMenuBar = new JMenuBar();
		helpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();

		try {
			File checkworkingPath =	new	File(workDirFile);
			if ( checkworkingPath.exists() == true ) {
				BufferedReader getContent =	new	BufferedReader(new FileReader(checkworkingPath));
				workDirFile = getContent.readLine();
			}			
		} catch (Exception e ) {
			System.out.println(" loading working path error");
		}			
		
		if(chooser == null) {
			chooser = new JFileChooser(workDirFile);
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle("Automatic Vehicle Routes Generator");

		jLabel1.setText("Automatic Vehicle Movements Generator");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.insets = new Insets(30, 0, 0, 108);
		jPanel1.add(jLabel1, gridBagConstraints);

		jtrOption.setText("Junction Turning Ratios");
		buttonGroup1.add(jtrOption);
		jtrOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				jtrOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(14, 10, 3, 61);
		jPanel1.add(jtrOption, gridBagConstraints);

		duaOption.setText("Dynamic Router");
		duaOption.setSelected(true);
		buttonGroup1.add(duaOption);
		duaOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				duaOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.insets = new Insets(17, 20, 0, 0);
		jPanel1.add(duaOption, gridBagConstraints);

		jLabel2.setText("Set Output File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 20, 0, 128);
		jPanel1.add(jLabel2, gridBagConstraints);

		jLabel3.setText("begin");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(4, 50, 0, 134);
		jPanel1.add(jLabel3, gridBagConstraints);

		jLabel4.setText("end");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.insets = new Insets(4, 90, 0, 24);
		jPanel1.add(jLabel4, gridBagConstraints);

		outputPath.setText(".rou.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 133;
		gridBagConstraints.insets = new Insets(4, 20, 100, 30);
		jPanel1.add(outputPath, gridBagConstraints);

		begin.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(4, 10, 100, 55);
		jPanel1.add(begin, gridBagConstraints);

		end.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 31;
		gridBagConstraints.insets = new Insets(4, 120, 100, 70);
		jPanel1.add(end, gridBagConstraints);

		netPath.setText(".net.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 134;
		gridBagConstraints.insets = new Insets(4, 20, 0, 30);
		jPanel1.add(netPath, gridBagConstraints);

		jLabel5.setText("Map File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.insets = new Insets(6, 20, 0, 24);
		jPanel1.add(jLabel5, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 136;
		gridBagConstraints.insets = new Insets(3, 50, 0, 30);
		jPanel1.add(flowjtrPath, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 136;
		gridBagConstraints.insets = new Insets(10, 50, 0, 30);
		jPanel1.add(turnPath, gridBagConstraints);

//		randompersecond.setText("10.0");
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 4;
//		gridBagConstraints.gridy = 2;
//		gridBagConstraints.ipadx = 19;
//		gridBagConstraints.insets = new Insets(3, 75, 0, 10);
//		jPanel1.add(randompersecond, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 136;
		gridBagConstraints.insets = new Insets(10, 75, 0, 43);
		jPanel1.add(tripsPath, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 136;
		gridBagConstraints.insets = new Insets(10, 75, 0, 43);
		jPanel1.add(flowduaPath, gridBagConstraints);

		OKButton.setText("OK");
		//OKButton.setPreferredSize(new Dimension(50,23));
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt,filename,osenv);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 19;
		gridBagConstraints.insets = new Insets(10, 20, 40, 0);
		jPanel1.add(OKButton, gridBagConstraints);

		cancelButton.setText("Cancel");
		//cancelButton.setPreferredSize(new Dimension(50,23));
		cancelButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.insets = new Insets(10, 10, 40, 57);
		jPanel1.add(cancelButton, gridBagConstraints);

		jLabel6.setText("Specify simulation interval");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 50, 0, 16);
		jPanel1.add(jLabel6, gridBagConstraints);

		openjtrFlow.setText("...");
		//openjtrFlow.setPreferredSize(new Dimension(45,23));
		openjtrFlow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openjtrFlowActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(3, 50, 0, 0);
		jPanel1.add(openjtrFlow, gridBagConstraints);

		openTurn.setText("...");
		//openTurn.setPreferredSize(new Dimension(45,23));
		openTurn.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openTurnActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 50, 0, 0);
		jPanel1.add(openTurn, gridBagConstraints);

		openNet.setText("...");
		//openNet.setPreferredSize(new Dimension(45,23));
		openNet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openNetActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 110, 0, 0);
		jPanel1.add(openNet, gridBagConstraints);

		openOutput.setText("...");
		//openOutput.setPreferredSize(new Dimension(45,23));
		openOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openOutputActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 110, 100, 0);
		jPanel1.add(openOutput, gridBagConstraints);

		openTrips.setText("...");
		//openTrips.setPreferredSize(new Dimension(45,23));
		openTrips.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openTripsActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 5;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 90, 0, 13);
		jPanel1.add(openTrips, gridBagConstraints);

		openduaFlow.setText("...");
		//openduaFlow.setPreferredSize(new Dimension(45,23));
		openduaFlow.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openduaFlowActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 5;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 90, 0, 13);
		jPanel1.add(openduaFlow, gridBagConstraints);

		jLabel7.setText("Flow definitions file");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(3, 20, 4, 102);
		jPanel1.add(jLabel7, gridBagConstraints);

		jLabel8.setText("Turn definitions file");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 20, 4, 102);
		jPanel1.add(jLabel8, gridBagConstraints);

//		randompersecOption.setText("Starting Vehicles");
//		randompersecOption.setSelected(true);
//		buttonGroup2.add(randompersecOption);
//		randompersecOption.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				randompersecOptionActionPerformed(evt);
//			}
//		});
//
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 3;
//		gridBagConstraints.gridy = 2;
//		gridBagConstraints.gridwidth = 2;
//		gridBagConstraints.gridheight = 2;
//		gridBagConstraints.insets = new Insets(3, 40, 26, 77);
//		jPanel1.add(randompersecOption, gridBagConstraints);

		flowDefOption.setText("Using flow definitions file");
		buttonGroup2.add(flowDefOption);
		flowDefOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				flowDefOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(10, 40, 26, 54);
		jPanel1.add(flowDefOption, gridBagConstraints);

		tripDefOption.setText("Using trip definitions file");
		buttonGroup2.add(tripDefOption);
		tripDefOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				tripDefOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(10, 40, 30, 59);
		jPanel1.add(tripDefOption, gridBagConstraints);

		bothOption.setText("Using both flow and trip definitions");
		buttonGroup2.add(bothOption);
		bothOption.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				bothOptionActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 40, 0, 0);
		jPanel1.add(bothOption, gridBagConstraints);

		statusText.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(statusText,
		JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipadx = 670;
		gridBagConstraints.ipady = 74;
		gridBagConstraints.insets = new Insets(14, 20, 0, 13);
		jPanel1.add(scrollPane, gridBagConstraints);

		getContentPane().add(jPanel1, BorderLayout.CENTER);

		helpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		helpMenu.add(quickHelpMenu);
		/*Quick help is disabled for now*/
		//jMenuBar.add(helpMenu);
		setJMenuBar(jMenuBar);
		pack();
		setVisible(true);
	}

	private void OKButtonActionPerformed(ActionEvent evt, String sumoPath, int envos) {
		String line;
		String cmd;
		String options = "";

		if (option == 0){
			if (osenv == 0) {
				options = System.getProperty("file.separator") + "jtrrouter" + "\"";
			    options += " -n " + "\"" + netPath.getText() + "\"";
			    options += " -f " + "\"" + flowjtrPath.getText() + "\"";
			    options += " -t " + "\"" + turnPath.getText() + "\"";  
			}
			else
				options = "jtrrouter -n " + netPath.getText() + " -f " + flowjtrPath.getText() + " -t " + turnPath.getText();
		}
		else if (option == 1){
			if (osenv == 0) {
				options = System.getProperty("file.separator") + "duarouter.exe" + "\"";
				options += " -n " + "\"" + netPath.getText() + "\"";
			}
			else
				options = "duarouter -n " + netPath.getText();
			
			if (duaoption == 0){
				options+= " -R ";
				options+= randompersecond.getText();				
			}
			else if (duaoption == 1){
				if (osenv == 0) {
					options+= " -f ";
					options+= "\"" + flowduaPath.getText() + "\"";			
				}
				else {
					options+= " -f ";
					options+= flowduaPath.getText();					
				}				
			}
			else if (duaoption == 2){
				if (osenv == 0) {
					options+= " -t ";
					options+= "\"" + tripsPath.getText() + "\"";
				}
				else {
					options+= " -t ";
					options+= tripsPath.getText();					
				}				
			}
			else if (duaoption == 3){
				if (osenv == 0) {
					options+= " -f ";
					options+= "\"" + flowduaPath.getText() + "\"";
					options+= " -t ";
					options+= "\"" + tripsPath.getText() + "\"";
				}
				else {
					options+= " -f ";
					options+= flowduaPath.getText();
					options+= " -t ";
					options+= tripsPath.getText();					
				}				
			}
		}

		/*Need to do OS environment check*/
		if (osenv == 0){
			//cmd = "cmd /c " + sumoPath + options + " -v -o " + outputPath.getText() + " -b " + begin.getText() + " -e " + end.getText() + " --continue-on-unbuild" + " -W";
			 
			cmd = "\"" + sumoPath + options + " -v -o ";
			cmd += "\"" + outputPath.getText() + "\"";;
			cmd += " -b " + begin.getText();
			cmd += " -e " + end.getText();
			cmd += " --continue-on-unbuild" + " -W";
//			System.out.println(cmd);
		}
		else {
			//System.out.println(duaoption);
			if (duaoption == 0){
//				cmd = "sumo-" + options + " -v -o " + outputPath.getText() + " -b " + begin.getText() + " -e " + end.getText() + " --continue-on-unbuild" + " -W";
				cmd = options + " -v -o " + outputPath.getText() + " -b " + begin.getText() + " -e " + end.getText() + " --continue-on-unbuild" + " -W";
			}
			else {
//				cmd = "sumo-" + options + " -v -o " + outputPath.getText() + " -b " + begin.getText() + " -e " + end.getText() + " --continue-on-unbuild";
				cmd = options + " -v -o " + outputPath.getText() + " -b " + begin.getText() + " -e " + end.getText() + " --continue-on-unbuild";
			}			
		}

		//System.out.println(cmd);
		try {
			Process process = Runtime.getRuntime().exec(cmd);
			
			new Thread(new SubTask(process.getInputStream())).start();
		    new Thread(new SubTask(process.getErrorStream())).start();
			process.waitFor();
			
//			BufferedReader buf = new BufferedReader(new InputStreamReader(process.getInputStream()));
//			BufferedReader buf2 = new BufferedReader(new InputStreamReader(process.getErrorStream()));
//					
//			while ((line = buf.readLine()) != null){
//				statusText.append(line + "\n");
//				statusText.setCaretPosition(statusText.getDocument().getLength());
//			}
//			while ((line = buf2.readLine()) != null){
//				statusText.append(line + "\n");
//				statusText.setCaretPosition(statusText.getDocument().getLength());
//			}
			process.destroy();
//			buf.close();
//			buf2.close();
		}
		catch (IOException err) {
			err.printStackTrace();
		}
		catch (Exception err) {
			err.printStackTrace();
		}		
	}
	
	class SubTask implements Runnable {
		  public SubTask(InputStream istream)
		  {
		    if (istream == null)
		      iReader = new BufferedReader(new InputStreamReader(System.in));
		    else
		      iReader = new BufferedReader(new InputStreamReader(istream));
		  }
		 
		  public void run()
		  {
		    try {
		      String input = iReader.readLine();
		      while (input != null)
		      {
//		        System.out.println(input);
				statusText.append(input + "\n");
				statusText.setCaretPosition(statusText.getDocument().getLength());
		        input = iReader.readLine();
		      }
		    }
		    catch (IOException ioe) {
		    }
		  }
		 
		  private BufferedReader iReader;
	}

	private void cancelButtonActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void openOutputActionPerformed(ActionEvent evt) {
		String extension = ".rou.xml";
		String openPath = "untitled";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showDialog(this,"Create");
		if(returnVal == JFileChooser.APPROVE_OPTION) {		 	
			openPath=chooser.getSelectedFile().getPath();
			if ( openPath.indexOf(extension) >= 0 )
				;
			else 
				openPath=chooser.getSelectedFile().getPath() + extension;	
		outputPath.setText( openPath );
		}
	}

	private void openNetActionPerformed(ActionEvent evt) {
		String extension = ".net.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		netPath.setText(chooser.getSelectedFile().getPath());
		
		try {
			PrintWriter workPath;			
			workPath = new PrintWriter(new	BufferedWriter(new FileWriter("workPath.txt")));
			workPath.println( getDirPATH(chooser.getSelectedFile().getPath()) );			
			workPath.close();			
		}
		catch (Exception e) {
			System.out.println(" writing working path error");
		}
	}

	private void openduaFlowActionPerformed(ActionEvent evt) {
		String extension = ".flow.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		flowduaPath.setText(chooser.getSelectedFile().getPath());
	}

	private void openTripsActionPerformed(ActionEvent evt) {
		String extension = ".trips.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		tripsPath.setText(chooser.getSelectedFile().getPath());
	}

	private void openTurnActionPerformed(ActionEvent evt) {
		String extension = ".turn.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		turnPath.setText(chooser.getSelectedFile().getPath());
	}

	private void openjtrFlowActionPerformed(ActionEvent evt) {
		String extension = ".flow.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		flowjtrPath.setText(chooser.getSelectedFile().getPath());
	}

	public String getDirPATH(String l) {
		String result = "";
		int length = l.length();
		int start = 0;
		int end = l.length()-1;
		
		while (end >= 0) {
			if ((l.charAt(end) == '\\') || (l.charAt(end) == '\n') ) {
				break;
			} else
				end--;
		}
//		System.out.println( start+1 +  " " + end + "   " + l);
		return (l.substring(start, end+1));
	}
	
	private void randompersecOptionActionPerformed(ActionEvent evt) {
		duaoption = 0;
	}
	private void flowDefOptionActionPerformed(ActionEvent evt) {
		duaoption = 1;
	}
	private void tripDefOptionActionPerformed(ActionEvent evt) {
		duaoption = 2;
	}
	private void bothOptionActionPerformed(ActionEvent evt) {
		duaoption = 3;
	}

	private void jtrOptionActionPerformed(ActionEvent evt) {
		option = 0;
	}
	private void duaOptionActionPerformed(ActionEvent evt) {
		option = 1;
	}
}


