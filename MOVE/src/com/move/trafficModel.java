package com.move;
import javax.swing.*;

import com.move.ns2Part.DynamicMobilityTCLEditor;
import com.move.ns2Part.TCLEditor;
import com.move.ns2Part.runNS;
import com.move.ns2Part.runNam;
import com.move.qualnetPart.qualnetEditor;
import com.move.qualnetPart.runQualnet;

import java.awt.*;
import java.awt.event.*;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney
*/

public class trafficModel extends JFrame {

	private JMenu file;
	private JMenu help;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel2;
	private JLabel jLabel22;
	private JLabel jLabel24;
	private JLabel jLabel6;
	private JLabel jLabel8;
	private JLabel jLabel9;
	private JLabel jLabe20;
	private JMenuBar jMenuBar;
	private JButton staticNsEditorButton;
	private JButton dynamicNsEditorButton;
	private JButton qualnetEditorButton;
	private JMenuItem quickHelp;
	private JMenuItem quitMenu;
	private JButton runNSButton;
	private JButton runNamButton;
	private JButton runQualnetButton;
	
	//private javax.swing.JPanel staticMapGeneratorPanel;
	
	public trafficModel() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;
		//staticMapGeneratorPanel = new javax.swing.JPanel();
		
		jLabel10 = new JLabel();
		jLabel2 = new JLabel();
		staticNsEditorButton = new JButton();
		dynamicNsEditorButton = new JButton();
		runNSButton = new JButton();
		runNamButton = new JButton();
		jLabel6 = new JLabel();
		qualnetEditorButton = new JButton();
		runQualnetButton = new JButton();
		jLabel24 = new JLabel();
		jLabel22 = new JLabel();
		jLabe20 = new JLabel();
		jLabel9 = new JLabel();
		jLabel8 = new JLabel();
		jLabel11 = new JLabel();
		jMenuBar = new JMenuBar();
		file = new JMenu();
		quitMenu = new JMenuItem();
		help = new JMenu();
		quickHelp = new JMenuItem();

		getContentPane().setLayout(new GridBagLayout());

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("Traffic Model Generator");
		jLabel10.setFont(new Font("Dialog", 1, 24));
		jLabel10.setForeground(new Color(255, 0, 51));
		jLabel10.setText("Traffic Model Generator for VANET");
		
		//staticMapGeneratorPanel.setBackground(new java.awt.Color(255, 255, 255));
		//staticMapGeneratorPanel.setBorder(javax.swing.BorderFactory.createTitledBorder(null, "Static Mobility", javax.swing.border.TitledBorder.DEFAULT_JUSTIFICATION, javax.swing.border.TitledBorder.DEFAULT_POSITION, new java.awt.Font("Arial", 0, 11), new java.awt.Color(0, 0, 204))); // NOI18N
        //staticMapGeneratorPanel.setLayout(new java.awt.GridBagLayout());
		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 10;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 0, 37);
		getContentPane().add(jLabel10, gridBagConstraints);

		jLabel2.setFont(new Font("Arial", 1, 14));
		jLabel2.setForeground(new Color(0, 51, 153));
		jLabel2.setText("NS-2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(18, 10, 0, 0);
		getContentPane().add(jLabel2, gridBagConstraints);

		staticNsEditorButton.setText("Static Mobility");		
		staticNsEditorButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				staticNsEditorButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();		
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;		
		gridBagConstraints.ipadx = 16;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 30, 0, 0);
		getContentPane().add(staticNsEditorButton, gridBagConstraints);

		dynamicNsEditorButton.setText("Dynamic Mobility");	
		dynamicNsEditorButton.setEnabled( true );
		dynamicNsEditorButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				dynamicNsEditorButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();		
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;		
		gridBagConstraints.ipadx = 0;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 30, 0, 0);
		getContentPane().add(dynamicNsEditorButton, gridBagConstraints);
		
		//staticMapGeneratorPanel.add(nsEditorButton, gridBagConstraints);
		//add(staticMapGeneratorPanel, gridBagConstraints);
		
		runNSButton.setText("Run NS-2");
		//runNSButton.setPreferredSize(new Dimension(100, 30));
		runNSButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				runNSButtonActionPerformed(evt);
			}
		});
		//
		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 30, 0, 0);
		getContentPane().add(runNSButton, gridBagConstraints);

		runNamButton.setText("Run Nam");		
		runNamButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				runNamButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 50;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 0, 0);
		getContentPane().add(runNamButton, gridBagConstraints);

		jLabel6.setFont(new Font("Arial", 1, 14));
		jLabel6.setForeground(new Color(0, 51, 153));
		jLabel6.setText("Qualnet");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 10, 0, 0);
		getContentPane().add(jLabel6, gridBagConstraints);

		qualnetEditorButton.setText("Editor");
		qualnetEditorButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				qualnetEditorButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 9;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 69;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 30, 0, 0);
		getContentPane().add(qualnetEditorButton, gridBagConstraints);

		runQualnetButton.setText("Run Qualnet");
		runQualnetButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				runQualnetButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 37;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 30, 35, 0);
		getContentPane().add(runQualnetButton, gridBagConstraints);

		//description
	
		jLabel8.setText("Static Traffic Model Generator for NS-2 (w/o TraCI)");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 2;		
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(13, 49, 0, 10);
		getContentPane().add(jLabel8, gridBagConstraints);
		
		jLabe20.setText("Dynamic Traffic Model Generator for NS-2 (w TraCI)");		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 3;			
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(2, 49, 0, 10);
		getContentPane().add(jLabe20, gridBagConstraints);
		
		jLabel9.setText("Run NS-2 in console");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 4;		
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 49, 0, 0);
		getContentPane().add(jLabel9, gridBagConstraints);
		
		jLabel11.setText("Visualize NS-2");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(3, 49, 0, 0);
		getContentPane().add(jLabel11, gridBagConstraints);

		jLabel22.setText("Traffic Model Generator for Qualnet");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 9;		
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(14, 49, 0, 0);
		getContentPane().add(jLabel22, gridBagConstraints);
		
		jLabel24.setText("Run Qualnet in console");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 11;		
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(4, 49, 0, 0);
		getContentPane().add(jLabel24, gridBagConstraints);

	
		
		
		file.setText("File");
		quitMenu.setText("Quit");
		quitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quitMenuActionPerformed(evt);
			}
		});

		file.add(quitMenu);

		jMenuBar.add(file);

		help.setText("Help");
		quickHelp.setText("Quick Help");
		quickHelp.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpActionPerformed(evt);
			}
		});

		help.add(quickHelp);
/*Quick help is disabled for now*/
		//jMenuBar.add(help);
		
		pack();
		setJMenuBar(jMenuBar);
		setVisible(true);
	}


	private void runQualnetButtonActionPerformed(ActionEvent evt) {
		runQualnet openQual = new runQualnet();
	}

	private void qualnetEditorButtonActionPerformed(ActionEvent evt) {
		qualnetEditor openQualEdit = new qualnetEditor();
	}

	private void runNSButtonActionPerformed(ActionEvent evt) {
		runNS openNS = new runNS();
	}

	private void runNamButtonActionPerformed(ActionEvent evt) {
		runNam openNam = new runNam();
	}

	private void staticNsEditorButtonActionPerformed(ActionEvent evt) {
		TCLEditor openTCL = new TCLEditor();
	}
	
	private void dynamicNsEditorButtonActionPerformed(ActionEvent evt) {
		DynamicMobilityTCLEditor openTCL = new DynamicMobilityTCLEditor();
	}

	private void quickHelpActionPerformed(ActionEvent evt) {
		JOptionPane.showMessageDialog(this, "Not yet available");
	}

	private void quitMenuActionPerformed(ActionEvent evt) {
		this.dispose();
	}


	public static void main(String args[]) {

		new trafficModel();

	}
}
