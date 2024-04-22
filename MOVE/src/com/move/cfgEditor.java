package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;

public class cfgEditor extends JFrame {
	static JFileChooser chooser;
	private String filename = "untitled";
	private static int netstateflag = 0;

	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel12;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JSeparator jSeparator1;

	private JCheckBox netstateCheck;
	private JMenuItem newMenu;
	private JMenuItem openMenu;
	private JButton openNet;
	private JButton openOutput;
	private JButton openRoute;

	private JTextField routePath;
	private JTextField netPath;
	private JTextField outputPath;
	private JTextField begin;
	private JTextField end;

	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;

	public cfgEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jLabel1 = new JLabel();
		netPath = new JTextField();
		routePath = new JTextField();
		outputPath = new JTextField();
		jLabel2 = new JLabel();
		jLabel3 = new JLabel();
		jLabel4 = new JLabel();
		jLabel10 = new JLabel();
		jLabel11 = new JLabel();
		jLabel12 = new JLabel();
		begin = new JTextField();
		end = new JTextField();
		openNet = new JButton();
		openRoute = new JButton();
		openOutput = new JButton();
		netstateCheck = new JCheckBox();
		jMenuBar = new JMenuBar();
		fileMenu = new JMenu();
		newMenu = new JMenuItem();
		openMenu = new JMenuItem();
		saveMenu = new JMenuItem();
		saveAsMenu = new JMenuItem();
		jSeparator1 = new JSeparator();
		quitMenu = new JMenuItem();
		helpMenu = new JMenu();
		quickHelpMenu = new JMenuItem();

		if(chooser == null) {
			chooser = new JFileChooser("Open File");
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle("Traffic Simulation Configurations Editor");

		jLabel1.setText("Traffic Simulator Configurations Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.insets = new Insets(10, 104, 0, 184);
		jPanel1.add(jLabel1, gridBagConstraints);

		netPath.setText(".net.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 114;
		gridBagConstraints.insets = new Insets(14, 74, 26, 30);
		jPanel1.add(netPath, gridBagConstraints);

		routePath.setText(".rou.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 113;
		gridBagConstraints.insets = new Insets(14, 74, 70, 30);
		jPanel1.add(routePath, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 156;
		gridBagConstraints.insets = new Insets(10, 20, 28, 30);
		jPanel1.add(outputPath, gridBagConstraints);

		jLabel2.setText("Input Files");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(34, 20, 0, 133);
		jPanel1.add(jLabel2, gridBagConstraints);

		jLabel3.setText("Map File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(14, 20, 0, 0);
		jPanel1.add(jLabel3, gridBagConstraints);

		jLabel4.setText("Routes File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(14, 20, 0, 128);
		jPanel1.add(jLabel4, gridBagConstraints);

		jLabel10.setText("Simulation Interval");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(34, 20, 0, 116);
		jPanel1.add(jLabel10, gridBagConstraints);

		jLabel11.setText("Begin");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(14, 20, 0, 0);
		jPanel1.add(jLabel11, gridBagConstraints);

		jLabel12.setText("End");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.insets = new Insets(14, 20, 0, 11);
		jPanel1.add(jLabel12, gridBagConstraints);

		begin.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 99;
		gridBagConstraints.insets = new Insets(14, 58, 26, 21);
		jPanel1.add(begin, gridBagConstraints);

		end.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 71;
		gridBagConstraints.insets = new Insets(14, 58, 70, 21);
		jPanel1.add(end, gridBagConstraints);

		openNet.setText("...");
		//openNet.setPreferredSize(new Dimension(45,23));
		openNet.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openNetActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(14, 90, 26, 0);
		jPanel1.add(openNet, gridBagConstraints);

		openRoute.setText("...");
		//openRoute.setPreferredSize(new Dimension(45,23));
		openRoute.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openRouteActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(14, 90, 70, 0);
		jPanel1.add(openRoute, gridBagConstraints);

		openOutput.setText("...");
		//openOutput.setPreferredSize(new Dimension(45,23));
		openOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openOutputActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 114, 28, 0);
		jPanel1.add(openOutput, gridBagConstraints);

		netstateCheck.setText("Set Output (Trace File)");
		netstateCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				parkingCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = -4;
		gridBagConstraints.insets = new Insets(54, 20, 0, 39);
		jPanel1.add(netstateCheck, gridBagConstraints);

		getContentPane().add(jPanel1, BorderLayout.CENTER);

		fileMenu.setText("File");
		newMenu.setText("New");
		newMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				newMenuActionPerformed(evt);
			}
		});

		fileMenu.add(newMenu);
		/*
		openMenu.setText("Open");
		openMenu.addActionListener(new ActionListener() {
		public void actionPerformed(ActionEvent evt) {
		openMenuActionPerformed(evt);
		}
		});
		fileMenu.add(openMenu);
		*/

		saveMenu.setText("Save");
		saveMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				saveMenuActionPerformed(evt);
			}
		});

		fileMenu.add(saveMenu);

		saveAsMenu.setText("Save As");
		saveAsMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				saveAsMenuActionPerformed(evt);
			}
		});

		fileMenu.add(saveAsMenu);

		fileMenu.add(jSeparator1);

		quitMenu.setText("Quit");
		quitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quitMenuActionPerformed(evt);
			}
		});

		fileMenu.add(quitMenu);

		jMenuBar.add(fileMenu);

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

	private void parkingCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			netstateflag = 0;
		}
		else {
			netstateflag = 1;
		}
	}

	private void openRouteActionPerformed(ActionEvent evt) {
		String extension = ".rou.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		routePath.setText(chooser.getSelectedFile().getPath());
	}

	private void openNetActionPerformed(ActionEvent evt) {
		String extension = ".net.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		netPath.setText(chooser.getSelectedFile().getPath());
	}

	private void openOutputActionPerformed(ActionEvent evt) {
		String extension = ".sumo.tr";
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

	private void newMenuActionPerformed(ActionEvent evt) {
		begin.setText("0");
		end.setText("");
		outputPath.setText("");
		netPath.setText("");
		routePath.setText("");
		filename = "untitled";
	}
	/*
	private void openMenuActionPerformed(ActionEvent evt) {
	int returnVal = chooser.showOpenDialog(this);
	if(returnVal == JFileChooser.APPROVE_OPTION)
	filename=chooser.getSelectedFile().getPath();
	}
	*/

	private void saveFile() {
		if (netstateflag == 0){
			construct cfgconstruction = new construct(netPath.getText(),routePath.getText(),new String(""),filename,begin.getText(),end.getText());
		}
		else{
			construct cfgconstruction = new construct(netPath.getText(),routePath.getText(),outputPath.getText(),filename,begin.getText(),end.getText());
		}
	}
	
	private void saveMenuActionPerformed(ActionEvent evt) {		
		if (filename == "untitled"){
			String extension = ".sumo.cfg";
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
		else {
			saveFile();
		}
	}

	private void saveAsMenuActionPerformed(ActionEvent evt) {
		String extension = ".sumo.cfg";
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

	private void quitMenuActionPerformed(ActionEvent evt) {
		this.dispose();
	}

	private void quickHelpMenuActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}
}

