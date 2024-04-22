package com.move;
import javax.swing.*;
import java.awt.event.*;
import java.awt.*;

public class netcEditor extends JFrame {
	static JFileChooser chooser;
	private String filename = "untitled";
	private static int typefileflag = 0;

	private JTextField edgePath;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel10;
	private JLabel jLabel12;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JLabel jLabel5;
	private JLabel jLabel6;
	private JLabel jLabel7;
	private JLabel jLabel8;
	private JLabel jLabel9;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JSeparator jSeparator1;
	private JTextField lanenumber;
	private JMenuItem newMenu;
	private JTextField nodePath;
	private JButton openEdge;
	private JMenuItem openMenu;
	private JButton openNode;
	private JButton openOutput;
	private JButton openTypes;
	private JTextField outputPath;
	private JTextField priority;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;
	private JTextField speed;
	private JCheckBox typeDefCheck;
	private JTextField typePath;

	public netcEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jLabel1 = new JLabel();
		jLabel2 = new JLabel();
		jLabel3 = new JLabel();
		nodePath = new JTextField();
		edgePath = new JTextField();
		outputPath = new JTextField();
		jLabel4 = new JLabel();
		jLabel5 = new JLabel();
		openNode = new JButton();
		openEdge = new JButton();
		openOutput = new JButton();
		typeDefCheck = new JCheckBox();
		jLabel12 = new JLabel();
		typePath = new JTextField();
		openTypes = new JButton();
		jLabel6 = new JLabel();
		jLabel7 = new JLabel();
		lanenumber = new JTextField();
		speed = new JTextField();
		priority = new JTextField();
		jLabel8 = new JLabel();
		jLabel9 = new JLabel();
		jLabel10 = new JLabel();
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
		setTitle("Map Configurations Editor");

		jLabel1.setText("Map Configurations Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.insets = new Insets(20, 142, 0, 23);
		jPanel1.add(jLabel1, gridBagConstraints);

		jLabel2.setText("Specify Input and Output Files");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(26, 20, 0, 202);
		jPanel1.add(jLabel2, gridBagConstraints);

		jLabel3.setText("Nodes File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(14, 20, 4, 0);
		jPanel1.add(jLabel3, gridBagConstraints);

		nodePath.setText(".nod.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.insets = new Insets(14, 12, 0, 149);
		jPanel1.add(nodePath, gridBagConstraints);

		edgePath.setText(".edg.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.insets = new Insets(10, 12, 44, 149);
		jPanel1.add(edgePath, gridBagConstraints);

		outputPath.setText(".net.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 114;
		gridBagConstraints.insets = new Insets(4, 20, 23, 209);
		jPanel1.add(outputPath, gridBagConstraints);

		jLabel4.setText("Roads File");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.insets = new Insets(10, 20, 48, 0);
		jPanel1.add(jLabel4, gridBagConstraints);

		jLabel5.setText("Set output file name");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 20, 0, 256);
		jPanel1.add(jLabel5, gridBagConstraints);

		openNode.setText("...");
		//openNode.setPreferredSize(new Dimension(45,23));
		openNode.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openNodeActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(14, 162, 0, 119);
		jPanel1.add(openNode, gridBagConstraints);

		openEdge.setText("...");
		//openEdge.setPreferredSize(new Dimension(45,23));
		openEdge.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openEdgeActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 162, 44, 119);
		jPanel1.add(openEdge, gridBagConstraints);

		openOutput.setText("...");
		//openOutput.setPreferredSize(new Dimension(45,23));
		openOutput.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openOutputActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 102, 23, 179);
		jPanel1.add(openOutput, gridBagConstraints);

		typeDefCheck.setText("Using road definitions from types file");
		typeDefCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				typeDefCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(40, 10, 10, 147);
		jPanel1.add(typeDefCheck, gridBagConstraints);

		jLabel12.setText("If yes, specify the type files");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(70, 20, 24, 217);
		jPanel1.add(jLabel12, gridBagConstraints);

		typePath.setText("types.xml");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 105;
		gridBagConstraints.insets = new Insets(16, 20, 0, 209);
		jPanel1.add(typePath, gridBagConstraints);

		openTypes.setText("...");
		//openTypes.setPreferredSize(new Dimension(45,23));
		openTypes.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				openTypesActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = -13;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(16, 102, 0, 179);
		jPanel1.add(openTypes, gridBagConstraints);

		jLabel6.setText("Road Defaults if road types are not defined");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(24, 232, 2, 23);
		jPanel1.add(jLabel6, gridBagConstraints);

		jLabel7.setText("and roads parameters are not inputted");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(2, 232, 16, 44);
		jPanel1.add(jLabel7, gridBagConstraints);

		lanenumber.setText("2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 119;
		gridBagConstraints.insets = new Insets(32, 31, 56, 23);
		jPanel1.add(lanenumber, gridBagConstraints);

		speed.setText("20");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 102;
		gridBagConstraints.insets = new Insets(28, 31, 26, 23);
		jPanel1.add(speed, gridBagConstraints);

		priority.setText("75");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 112;
		gridBagConstraints.insets = new Insets(58, 31, 32, 23);
		jPanel1.add(priority, gridBagConstraints);

		jLabel8.setText("Lane Numbers");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(32, 232, 60, 180);
		jPanel1.add(jLabel8, gridBagConstraints);

		jLabel9.setText("Max Speed");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 27;
		gridBagConstraints.insets = new Insets(28, 232, 30, 173);
		jPanel1.add(jLabel9, gridBagConstraints);

		jLabel10.setText("Roads priority");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.insets = new Insets(58, 232, 0, 0);
		jPanel1.add(jLabel10, gridBagConstraints);

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

	private void typeDefCheckItemStateChanged(ItemEvent evt) {
		if (evt.getStateChange() == ItemEvent.DESELECTED) {
			typefileflag = 0;
		}
		else {
			typefileflag = 1;
		}
	}

	private void newMenuActionPerformed(ActionEvent evt) {
		nodePath.setText("");
		edgePath.setText("");
		outputPath.setText("");
		typePath.setText("");
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
		if (typefileflag == 0){
			construct netcconstruction = new construct(filename,nodePath.getText(),edgePath.getText(),"",outputPath.getText(),1,"Unknown",lanenumber.getText(),speed.getText(),priority.getText());
		}
		else {
			construct netcconstruction = new construct(filename,nodePath.getText(),edgePath.getText(),typePath.getText(),outputPath.getText(),1,"Unknown",lanenumber.getText(),speed.getText(),priority.getText());
		}
	}
	
	private void saveMenuActionPerformed(ActionEvent evt) {
		if (filename == "untitled"){
			String extension = ".netc.cfg";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showSaveDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION){
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
		String extension = ".netc.cfg";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showSaveDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION){
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

	private void openTypesActionPerformed(ActionEvent evt) {
		String extension = ".type.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		typePath.setText(chooser.getSelectedFile().getPath());
	}

	private void openOutputActionPerformed(ActionEvent evt) {
		String extension = ".net.xml";
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

	private void openEdgeActionPerformed(ActionEvent evt) {
		String extension = ".edg.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		edgePath.setText(chooser.getSelectedFile().getPath());
	}

	private void openNodeActionPerformed(ActionEvent evt) {
		String extension = ".nod.xml";
		chooser.setFileFilter( new JaveFileFilter(extension) );
		int returnVal = chooser.showOpenDialog(this);
		if(returnVal == JFileChooser.APPROVE_OPTION)
		nodePath.setText(chooser.getSelectedFile().getPath());
	}


	public static void main(String args[]) {
		new netcEditor();
	}
}
