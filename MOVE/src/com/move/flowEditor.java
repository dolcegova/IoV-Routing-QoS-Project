package com.move;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.commonData.flowData;
import com.move.commonData.flowParameter;

import java.awt.*;
import java.awt.event.*;

public class flowEditor extends JFrame {

	private String filename = "untitled";
	private FlowDynamicTableModel mytable = new FlowDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;

	private JButton addRowButton;
	private JButton automaticButton;
	private JTextField beginDef;
	private JButton beginDefButton;
	private JTextField endDef;
	private JButton endDefButton;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JMenuBar jMenuBar1;
	private JPanel jPanel1;
	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;

	private JMenuItem newMenu;
	private JMenuItem openMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removeRowButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;
	private JTextField vehiclesDef;
	private JButton vehiclesDefButton;

	public flowEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		addRowButton = new JButton();
		removeRowButton = new JButton();
		jLabel1 = new JLabel();
		jScrollPane1 = new JScrollPane();

		jLabel2 = new JLabel();
		automaticButton = new JButton();
		beginDefButton = new JButton();
		jLabel3 = new JLabel();
		endDefButton = new JButton();
		beginDef = new JTextField();
		endDef = new JTextField();
		vehiclesDefButton = new JButton();
		vehiclesDef = new JTextField();
		jMenuBar1 = new JMenuBar();
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
			chooser = new JFileChooser("Flow XML File");
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle("Vehicle Flow Definitions Editor");

		addRowButton.setText("Add Flow");
		//addRowButton.setPreferredSize(new Dimension(79,23));
		addRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.ipadx = 36;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 90, 0, 0);
		jPanel1.add(addRowButton, gridBagConstraints);

		removeRowButton.setText("Remove Flow");
		//removeRowButton.setPreferredSize(new Dimension(79,23));
		removeRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 12;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(10, 90, 20, 0);
		jPanel1.add(removeRowButton, gridBagConstraints);

		jLabel1.setText("Vehicle Flows Definitions");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(20, 10, 0, 149);
		jPanel1.add(jLabel1, gridBagConstraints);


		table = new JTable(mytable);
		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 147;
		gridBagConstraints.ipady = -209;
		gridBagConstraints.insets = new Insets(14, 10, 0, 10);
		jPanel1.add(jScrollPane1, gridBagConstraints);

		jLabel2.setText("Assign Automatic Flow IDs");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 10, 0, 140);
		jPanel1.add(jLabel2, gridBagConstraints);

		automaticButton.setText("Automatic ID");
		//automaticButton.setPreferredSize(new Dimension(95,23));
		automaticButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				automaticButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 4;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 10, 0, 0);
		jPanel1.add(automaticButton, gridBagConstraints);

		beginDefButton.setText("begin");
		//beginDefButton.setPreferredSize(new Dimension(69,23));
		beginDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				beginDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 25;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 90, 0, 0);
		jPanel1.add(beginDefButton, gridBagConstraints);

		jLabel3.setText("Set Defaults");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.insets = new Insets(10, 90, 0, 21);
		jPanel1.add(jLabel3, gridBagConstraints);

		endDefButton.setText("end");
		//endDefButton.setPreferredSize(new Dimension(69,23));
		endDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				endDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 35;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 90, 0, 0);
		jPanel1.add(endDefButton, gridBagConstraints);

		beginDef.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 69;
		gridBagConstraints.insets = new Insets(4, 0, 0, 0);
		jPanel1.add(beginDef, gridBagConstraints);

		endDef.setText("1000");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 48;
		jPanel1.add(endDef, gridBagConstraints);

		vehiclesDefButton.setText("vehicles");
		//vehiclesDefButton.setPreferredSize(new Dimension(69,23));
		vehiclesDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				vehiclesDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 9;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 90, 19, 0);
		jPanel1.add(vehiclesDefButton, gridBagConstraints);

		vehiclesDef.setText("100");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 55;
		gridBagConstraints.insets = new Insets(0, 0, 19, 0);
		jPanel1.add(vehiclesDef, gridBagConstraints);

		getContentPane().add(jPanel1, BorderLayout.CENTER);

		fileMenu.setText("File");
		newMenu.setText("New");
		newMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				newMenuActionPerformed(evt);
			}
		});

		fileMenu.add(newMenu);

		openMenu.setText("Open");
		openMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				try{
					openMenuActionPerformed(evt);
				}
				catch (Exception e){
				}
			}
		});

		fileMenu.add(openMenu);

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

		jMenuBar1.add(fileMenu);

		helpMenu.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		helpMenu.add(quickHelpMenu);
		/*Quick help is disabled for now*/
		//jMenuBar1.add(helpMenu);

		setJMenuBar(jMenuBar1);

		pack();
		setVisible(true);
	}

	class FlowDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"ID","From Edge","To Edge","Begin","End","No Vehicles"};
		Vector vectorTableData = new Vector(10,2);

		public int getColumnCount() {
			return 6;
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

		private void newMenuActionPerformed(ActionEvent evt) {
			while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount()-1);
			filename = "untitled";
		}

		private void openMenuActionPerformed(ActionEvent evt) throws Exception{
			while (mytable.getRowCount() != 0)
			mytable.removeRow(mytable.getRowCount()-1);

			String extension = ".flow.xml";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			filename=chooser.getSelectedFile().getPath();

			Load l = new Load(filename);
			l.initFlows();
			l.readfile();
			flowData test = l.getFlowData();
			ArrayList list = test.returnData();

			for(int i =0;i<list.size();i++){
				flowParameter e = (flowParameter) list.get(i);
				Vector newRow = new Vector();

				newRow.addElement(e.getID());
				newRow.addElement(e.getFrom());
				newRow.addElement(e.getTo());
				newRow.addElement(e.getBegin());
				newRow.addElement(e.getEnd());
				newRow.addElement(e.getNo());

				mytable.addRow(newRow);
			}
		}

		private void saveFile() {
			flowData m = new flowData();
			flowParameter n;
			String[] temp = {"","","","","",""};
			for(int i = 0; i<mytable.getRowCount(); i++){
				for (int j=0; j<mytable.getColumnCount(); j++) {
					temp[j]=(String)mytable.getValueAt(i, j);
				}
				n = new flowParameter(temp[0],temp[1],temp[2],temp[3],temp[4],temp[5]);
				m.addFlows(n);
			}
			construct flowconstruction = new construct(m,filename);					
		}
		
		private void saveMenuActionPerformed(ActionEvent evt) {
			if (filename == "untitled") {
				String extension = ".flow.xml";
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
			String extension = ".flow.xml";
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

		private void addRowButtonActionPerformed(ActionEvent evt) {
			Vector newRow = new Vector();
			newRow.addElement(new String("")); //ID
			newRow.addElement(new String("")); //From
			newRow.addElement(new String("")); //To
			newRow.addElement(new String("0")); //Begin
			newRow.addElement(new String("")); //End
			newRow.addElement(new String("")); //Vehicles
			mytable.addRow(newRow);
		}

		private void removeRowButtonActionPerformed(ActionEvent evt) {
			if (mytable.getRowCount() > 0)
			mytable.removeRow(mytable.getRowCount()-1);
		}

		private void automaticButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<mytable.getRowCount(); i++)
			mytable.setValueAt(new String("flow"+i),i,0);
		}

		private void vehiclesDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(vehiclesDef.getText(),i,5);
		}

		private void endDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(endDef.getText(),i,4);
		}

		private void beginDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(beginDef.getText(),i,3);
		}

		public static void main(String args[]) {
			new flowEditor();
		}

	}
