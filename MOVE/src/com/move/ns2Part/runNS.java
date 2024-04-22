package com.move.ns2Part;
/**
*
* @author  Feliz Kristianto
* 
* revised by Chien-Ming Chou, 
* NCKU, CSIE, LENS lab
* 
* 
*/

import javax.swing.*;

import com.move.JaveFileFilter;
import com.move.setPath;

import java.awt.event.*;
import java.awt.*;
import java.io.*;


public class runNS extends JFrame {	
	private String filename = "untitled";
	static JFileChooser chooser;
	private JMenu HelpMenu;
	private JMenu FileMenu;
	private JButton OKButton;
	private JButton cancelButton;
	private JTextField nsPath;
	private JLabel jLabel1;
	private JLabel jLabel3;
	private JMenuBar jMenuBar1;
	private JButton openFile;
	private JMenuItem quickHelpMenu;
	private JMenuItem QuitMenu;
	private JMenuItem traCIconfigure;
	private JMenuItem setPathMenu;
	private JTextArea statusText;	
	
    private javax.swing.JLabel existingNet;
    private javax.swing.JTextField networkFile;
    private javax.swing.JButton networkButton;    
    private javax.swing.JTextField routesFile;
    private javax.swing.JButton routesFileButton;
    private javax.swing.JLabel existingRouteFile;
    
    private static int modeOption = 1; 		// 1 for static, 2 for dynamic 
    private JRadioButton staticModeOption;
    private JRadioButton dynamicModeOption;
    private ButtonGroup buttonGroup1;
    private int traciSetFlag = 0;
    private int sumoBinariesFlag = 0;
    private int osenv = 0;
    
	private String traciHostname 		= "unsetting";
	private String traciPort 		 	= "unsetting";
	private String traciPenetration		= "unsetting";
	private String traciSimulationTime	= "unsetting";
	private String sumoPath		 		= "unsetting";

	public runNS() {
		String osname = System.getProperty("os.name","").toLowerCase();
		if ( osname.startsWith("windows") ) {
		   // windows
		   osenv = 0;
		}	    
		else {
		   osenv = 1;
		}
		initComponents();		
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;
		jLabel1 = new JLabel();
		jLabel3 = new JLabel();
		nsPath = new JTextField();
		openFile = new JButton();
		statusText = new JTextArea();
		cancelButton = new JButton();
		OKButton = new JButton();
		jMenuBar1 = new JMenuBar();
		HelpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();
		QuitMenu = new JMenuItem();
		FileMenu = new JMenu();
		traCIconfigure = new JMenuItem();
		setPathMenu = new JMenuItem();
		
		existingNet = new javax.swing.JLabel();
		existingRouteFile = new javax.swing.JLabel();
		networkFile = new javax.swing.JTextField();
		networkButton = new javax.swing.JButton();		
		routesFile = new javax.swing.JTextField();
		routesFileButton = new javax.swing.JButton();
		
		buttonGroup1 = new ButtonGroup();
		staticModeOption = new JRadioButton();
		dynamicModeOption = new JRadioButton();

		getContentPane().setLayout(new GridBagLayout());
		
		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("Run NS-2 in console");
		jLabel1.setFont(new Font("Tahoma", 1, 14));
		jLabel1.setHorizontalAlignment(SwingConstants.CENTER);
		jLabel1.setText("Run NS-2 in background Console (without TraCI only)");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.insets = new Insets(20, -10, 10, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

//		staticModeOption.setText("Static Mobility Mode");
//		staticModeOption.setFont(new Font("Tahoma", 1, 11));
//		staticModeOption.setSelected(true);
//		buttonGroup1.add(staticModeOption);
//		staticModeOption.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				staticModeOptionActionPerformed(evt);
//			}
//		});
//
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 0;
//		gridBagConstraints.gridy = 0;
//		gridBagConstraints.gridwidth = 4;
//		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//		gridBagConstraints.insets = new Insets(100, 20, 0, 0);
//		getContentPane().add(staticModeOption, gridBagConstraints);
//		
//		dynamicModeOption.setText("Dynamic Mobility Mode");
//		dynamicModeOption.setEnabled( false );
//		dynamicModeOption.setFont(new Font("Tahoma", 1, 11));
//		buttonGroup1.add(dynamicModeOption);
//		dynamicModeOption.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				dynamicModeOptionActionPerformed(evt);
//			}
//		});
//
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 0;
//		gridBagConstraints.gridy = 0;
//		gridBagConstraints.gridwidth = 4;
//		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//		gridBagConstraints.insets = new Insets(100, 180, 0, 0);
//		getContentPane().add(dynamicModeOption, gridBagConstraints);
				
		jLabel3.setText("NS-2 script file:");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = 3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 30, 10, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		nsPath.setText("filename.tcl");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipadx = 250;
		gridBagConstraints.ipady = 1;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(10, -50, 10, 0);
		getContentPane().add(nsPath, gridBagConstraints);

		openFile.setText("...");
		openFile.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openFileActionPerformed(evt);
			}
		});

		openFile.setMaximumSize(new java.awt.Dimension(25, 25));
		openFile.setMinimumSize(new java.awt.Dimension(25, 25));
		openFile.setPreferredSize(new java.awt.Dimension(25, 25)); 
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 0, 10, 20);
		getContentPane().add(openFile, gridBagConstraints);
		
//        existingNet.setText("Specify Network File:");
//        existingNet.setEnabled(false);
//        gridBagConstraints = new java.awt.GridBagConstraints();
//        gridBagConstraints.gridx = 0;
//        gridBagConstraints.gridy = 2;
//        gridBagConstraints.ipady = 5;
//        gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//        gridBagConstraints.insets = new Insets(0, 20, 10, 0);
//        getContentPane().add(existingNet, gridBagConstraints);
//		
//        networkFile.setText("filename.net.xml");
//        networkFile.setEnabled(false);
//        gridBagConstraints = new java.awt.GridBagConstraints();        
//        gridBagConstraints.gridx = 1;
//        gridBagConstraints.gridy = 2;
//        gridBagConstraints.ipadx = 150;
//		gridBagConstraints.ipady = 1;
//        gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//        gridBagConstraints.insets = new Insets(0, 10, 10, 0);
//        getContentPane().add(networkFile, gridBagConstraints);
//
//        networkButton.setText("...");        
//        networkButton.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				openNetFileActionPerformed(evt);
//			}
//		});
//        
//        networkButton.setMaximumSize(new java.awt.Dimension(25, 25));
//        networkButton.setMinimumSize(new java.awt.Dimension(25, 25));
//        networkButton.setPreferredSize(new java.awt.Dimension(25, 25));
//        networkButton.setEnabled(false);
//        gridBagConstraints = new java.awt.GridBagConstraints();
//        gridBagConstraints.gridx = 2;
//        gridBagConstraints.gridy = 2;     
//        gridBagConstraints.ipady = -3;
//        gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//        gridBagConstraints.insets = new Insets(0, 0, 10, 20);
//        getContentPane().add(networkButton, gridBagConstraints);
//        
//        existingRouteFile.setText("Specify Routes File:");
//        existingRouteFile.setEnabled(false);
//        gridBagConstraints = new java.awt.GridBagConstraints();
//        gridBagConstraints.gridx = 0;
//        gridBagConstraints.gridy = 3;
//        gridBagConstraints.ipady = 5;
//        gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//        gridBagConstraints.insets = new Insets(0, 20, 10, 0);
//        getContentPane().add(existingRouteFile, gridBagConstraints);
//		
//        routesFile.setText("filename.rou.xml");
//        routesFile.setEnabled(false);
//        gridBagConstraints = new java.awt.GridBagConstraints();        
//        gridBagConstraints.gridx = 1;
//        gridBagConstraints.gridy = 3;
//        gridBagConstraints.ipadx = 150;
//		gridBagConstraints.ipady = 1;
//        gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//        gridBagConstraints.insets = new Insets(0, 10, 10, 0);
//        getContentPane().add(routesFile, gridBagConstraints);
//
//        routesFileButton.setText("...");        
//        routesFileButton.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				openRouFileActionPerformed(evt);
//			}
//		});
//        
//        routesFileButton.setMaximumSize(new java.awt.Dimension(25, 25));
//        routesFileButton.setMinimumSize(new java.awt.Dimension(25, 25));
//        routesFileButton.setPreferredSize(new java.awt.Dimension(25, 25));
//        routesFileButton.setEnabled(false);
//		gridBagConstraints = new GridBagConstraints();
//		gridBagConstraints.gridx = 2;
//		gridBagConstraints.gridy = 3;
//		gridBagConstraints.ipady = -3;
//		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
//		gridBagConstraints.insets = new Insets(0, 0, 10, 20);
//        getContentPane().add(routesFileButton, gridBagConstraints);
		
		statusText.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(statusText,
		JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
		JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 450;
		gridBagConstraints.ipady = 150;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 10, 20);
		getContentPane().add(scrollPane, gridBagConstraints);

		OKButton.setText("OK");
		OKButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				OKButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHEAST;
		gridBagConstraints.insets = new Insets(20, 20, 20, 0);
		getContentPane().add(OKButton, gridBagConstraints);
				
		cancelButton.setText("Cancel");
		cancelButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				cancelButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(20, 10, 20, 20);
		getContentPane().add(cancelButton, gridBagConstraints);

		
		/******************************************************** 
		 *   Menu part
		 ********************************************************/
		
		FileMenu.setText("File");
		
		setPathMenu.setText("Set SUMO Binaries Path");
		setPathMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				setPathMenuActionPerformed(evt);
			}
		});
		FileMenu.add(setPathMenu);
		
//		traCIconfigure.setText( "TraCI network configuration");
//		traCIconfigure.addActionListener(new ActionListener() {
//			public void actionPerformed(ActionEvent evt) {
//				try{					
//					traciNetworkConfigure(evt);
//				}
//				catch (Exception e){
//				};
//			}
//		});
//		FileMenu.add(traCIconfigure);
		
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
		pack();
		setVisible(true);
	}

	// refer from TraNS 
    private class CommandExecuter implements Runnable {
        private boolean active = true;
        private String command = "";
        
        CommandExecuter(String command) {
            this.command = command;
        }
        
        public void run() {
        	String line;
        	StringBuffer sb=new StringBuffer();
            try {
                Process sumoprocess = Runtime.getRuntime().exec(command);
                
                new Thread(new SubTask(sumoprocess.getInputStream())).start();
    		    new Thread(new SubTask(sumoprocess.getErrorStream())).start();
    		    sumoprocess.waitFor();
                
//			    BufferedReader buf2 = new BufferedReader(new InputStreamReader(sumoprocess.getErrorStream()));
//                BufferedReader buf = new BufferedReader(new InputStreamReader(sumoprocess.getInputStream()));
//                while ((line = buf.readLine()) != null){
//					statusText.append(line + "\n");
//					statusText.setCaretPosition(statusText.getDocument().getLength());
//				}				
//                while ((line = buf2.readLine()) != null){
//					statusText.append(line + "\n");
//					statusText.setCaretPosition(statusText.getDocument().getLength());
//				}
				sumoprocess.destroy();
//				buf.close();
//				buf2.close();				
            }
            catch (Exception err) {
				err.printStackTrace();
			}		            
        }
        
        public void stop() {
            active = false;
        }
    }	
	
	private void QuitMenuActionPerformed(ActionEvent evt) {
		this.dispose();
	}
	
	private void traciNetworkConfigure(ActionEvent evt) throws Exception{
		setTraCI open = new setTraCI();  			
	}
	
	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}
	
	private void staticModeOptionActionPerformed(ActionEvent evt) {
		modeOption = 1;
		existingNet.setEnabled(false);
		networkFile.setEnabled(false);
		networkButton.setEnabled(false);
		networkButton.setEnabled(false);
		existingRouteFile.setEnabled(false);
		routesFile.setEnabled(false);
		routesFileButton.setEnabled(false);
	}

	private void dynamicModeOptionActionPerformed(ActionEvent evt) {
		modeOption = 2;
		existingNet.setEnabled(true);
		networkFile.setEnabled(true);
		networkButton.setEnabled(true);
		networkButton.setEnabled(true);
		existingRouteFile.setEnabled(true);
		routesFile.setEnabled(true);
		routesFileButton.setEnabled(true);
	}	
	
	private void setPathMenuActionPerformed(ActionEvent evt) {
	
	    if (System.getProperty("os.name").startsWith("Windows")) {
		    // windows		    
		    setPath newsetpath = new setPath();
		}	    
	    else {			
			JOptionPane.showMessageDialog(this, "The current OS environment is set as Linux.\n" +
			"You do not need to set the SUMO binaries path.\n" +
			"It's already in your exported Path after installation.\n");
		}		
	}
	
	private void OKButtonActionPerformed(ActionEvent evt) {
		String line;
		String cmd = "unsetting";
		
		traciSetFlag = 0;
		sumoBinariesFlag = 0;
		try {
			File openfile	=	new	File("traciConfigure.txt");
			BufferedReader getContent	=	new	BufferedReader(new FileReader(openfile));
			
			traciSetFlag = 1;
			traciHostname = getContent.readLine();
			traciPort = getContent.readLine();
			traciPenetration = getContent.readLine();
			traciSimulationTime = getContent.readLine();				
		}
		catch (Exception e) {}
		
		
		if ( traciSetFlag == 0 && modeOption == 2){
			JOptionPane.showMessageDialog(this, "Please first set TraCI network configuration\n"); 		
		}
		else {
			try {
				File filterfile	=	new	File("path.txt");
				BufferedReader getContent	=	new	BufferedReader(new FileReader(filterfile));
				sumoBinariesFlag = 1;
				sumoPath = getContent.readLine();
			}
			catch (Exception e) {}
			
			if ( sumoBinariesFlag == 0 && osenv == 0){
				JOptionPane.showMessageDialog(this, "Please first set SUMO Binaries \n");
			}
			else {
				if (modeOption == 1) {
					try {
						cmd = "ns " + nsPath.getText();
						Process process = Runtime.getRuntime().exec(cmd);
						
						new Thread(new SubTask(process.getInputStream())).start();
					    new Thread(new SubTask(process.getErrorStream())).start();
						process.waitFor();
						
//						BufferedReader buf = new BufferedReader(new InputStreamReader(process.getInputStream()));
//						BufferedReader buf2 = new BufferedReader(new InputStreamReader(process.getErrorStream()));
//						while ((line = buf.readLine()) != null){
//							statusText.append(line + "\n");
//							statusText.setCaretPosition(statusText.getDocument().getLength());
//						}
//						
//						while ((line = buf2.readLine()) != null){
//							statusText.append(line + "\n");
//							statusText.setCaretPosition(statusText.getDocument().getLength());
//						}			
						process.destroy();
//						buf.close();
//						buf2.close();
					}
					catch (Exception err) {
						err.printStackTrace();
					}				
				}
				
				// modeOption == 2
				else {
					if(System.getProperty("os.name").startsWith("Windows")){
						//sumo -n TGR36061_crop.net.xml -r TGR36061_crop.rou.xml -b 0 -e 1000 --remote-port 8888 --penetration 1
						cmd = "\"" + sumoPath + System.getProperty("file.separator") + "sumo.exe" + "\"";
		                cmd += " -n " + "\"" + networkFile.getText() + "\"";
		                cmd += " -r " + "\"" + routesFile.getText() + "\"";
		                cmd += " -b " + "0";
		                cmd += " -e " + traciSimulationTime;
		                cmd += " --remote-port " + traciPort;
		                cmd += " --penetration " + traciPenetration;

						JOptionPane.showMessageDialog(this, "execution command: " + cmd + "\n"
							+ "Current MOVE not yet supported windows platform in dynamic mobility mode");
		            }
		            else{
		                //sumo -n TGR36061_crop.net.xml -r TGR36061_crop.rou.xml -b 0 -e 1000 --remote-port 8888 --penetration 1
		            	cmd = "sumo";
		            	cmd += " -n " + networkFile.getText();
		                cmd += " -r " + routesFile.getText();
		                cmd += " -b " + "0";
		                cmd += " -e " + traciSimulationTime;
		                cmd += " --remote-port " + traciPort;
		                cmd += " --penetration " + traciPenetration;
		                
		                CommandExecuter ce = new CommandExecuter(cmd);
		                new Thread(ce).start();
		                try {
		                    Thread.currentThread().sleep(1000);
		                } catch (InterruptedException ex) {
		                    ex.printStackTrace();
		                }
		                
		                // ns vanet_simulation.tcl
		                try {
		                	cmd =  "ns " + nsPath.getText();
		                	Process process = Runtime.getRuntime().exec(cmd);		
		                	
		                	new Thread(new SubTask(process.getInputStream())).start();
		        		    new Thread(new SubTask(process.getErrorStream())).start();
		        			process.waitFor();		                	
		                	
//		        			BufferedReader buf = new BufferedReader(new InputStreamReader(process.getInputStream()));
//		        			BufferedReader buf2 = new BufferedReader(new InputStreamReader(process.getErrorStream()));
//		        			while ((line = buf.readLine()) != null){
//		        				statusText.append(line + "\n");
//		        				statusText.setCaretPosition(statusText.getDocument().getLength());
//		        			}
//		        			
//		        			while ((line = buf2.readLine()) != null){
//		        				statusText.append(line + "\n");
//		        				statusText.setCaretPosition(statusText.getDocument().getLength());
//		        			}			
		        			process.destroy();
//		        			buf.close();
//		        			buf2.close();
		                }
		                catch (Exception err) {
							err.printStackTrace();
						}		                
		            }
				}
			}	
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
	
	private void openFileActionPerformed(ActionEvent evt) {
		String extension = ".tcl";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		nsPath.setText(chooser.getSelectedFile().getPath());
	}

	private void openNetFileActionPerformed(ActionEvent evt) {
		String extension = ".net.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		networkFile.setText(chooser.getSelectedFile().getPath());
	}
	
	private void openRouFileActionPerformed(ActionEvent evt) {
		String extension = ".rou.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		routesFile.setText(chooser.getSelectedFile().getPath());
	}
	
	public static void main(String args[]) {
		new runNS();
	}


}
