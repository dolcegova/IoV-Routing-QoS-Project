package com.move;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.commonData.typeData;
import com.move.commonData.typeParameter;

import java.awt.event.*;
import java.awt.*;

public class typeEditor extends JFrame {
	private String filename = "untitled";
	private TypeDynamicTableModel mytable = new TypeDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;

	private JButton addRowButton;
	private JButton automaticButton;
	private JTextField capacityDef;
	private JButton capacityDefButton;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;

	private JMenuItem newMenu;
	private JTextField nolanesDef;
	private JButton nolanesDefButton;
	private JMenuItem openMenu;
	private JCheckBox parkingCheck;
	private JTextField priorityDef;
	private JButton priorityDefButton;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removeRowButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;
	private JTextField speedDef;
	private JButton speedDefButton;

	public typeEditor() {
		initComponents();
	}


	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jScrollPane1 = new JScrollPane();

		jLabel1 = new JLabel();
		jLabel2 = new JLabel();
		speedDefButton = new JButton();
		speedDef = new JTextField();
		capacityDef = new JTextField();
		capacityDefButton = new JButton();
		removeRowButton = new JButton();
		addRowButton = new JButton();
		jLabel3 = new JLabel();
		automaticButton = new JButton();
		priorityDef = new JTextField();
		priorityDefButton = new JButton();
		nolanesDefButton = new JButton();
		nolanesDef = new JTextField();
		parkingCheck = new JCheckBox();
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
			chooser = new JFileChooser("Road Type XML File");
		}

		jPanel1.setLayout(new GridBagLayout());
		setTitle("Road Types Editor");

		table = new JTable(mytable);
		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.ipadx = 137;
		gridBagConstraints.ipady = -259;
		gridBagConstraints.insets = new Insets(14, 20, 0, 10);
		jPanel1.add(jScrollPane1, gridBagConstraints);

		jLabel1.setText("Road Types Definitions");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(20, 20, 0, 131);
		jPanel1.add(jLabel1, gridBagConstraints);

		jLabel2.setText("Set Defaults");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(10, 20, 0, 191);
		jPanel1.add(jLabel2, gridBagConstraints);

		speedDefButton.setText("speed");
		//speedDefButton.setPreferredSize(new Dimension(69,23));
		speedDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				speedDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 21;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 102, 0, 0);
		jPanel1.add(speedDefButton, gridBagConstraints);

		speedDef.setText("20.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(4, 0, 0, 0);
		jPanel1.add(speedDef, gridBagConstraints);

		capacityDef.setText("100.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 59;
		jPanel1.add(capacityDef, gridBagConstraints);

		capacityDefButton.setText("capacity");
		//capacityDefButton.setPreferredSize(new Dimension(69,23));
		capacityDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				capacityDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 8;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 102, 0, 0);
		jPanel1.add(capacityDefButton, gridBagConstraints);

		removeRowButton.setText("Remove Type");
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
		gridBagConstraints.insets = new Insets(10, 130, 16, 0);
		jPanel1.add(removeRowButton, gridBagConstraints);

		addRowButton.setText("Add Type");
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
		gridBagConstraints.insets = new Insets(20, 130, 0, 0);
		jPanel1.add(addRowButton, gridBagConstraints);

		jLabel3.setText("Assign Automatic Road Type ID");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(20, 20, 4, 84);
		jPanel1.add(jLabel3, gridBagConstraints);

		automaticButton.setText("Automatic ID");
		//automaticButton.setPreferredSize(new Dimension(95,23));
		automaticButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				automaticButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 14;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 20, 0, 140);
		jPanel1.add(automaticButton, gridBagConstraints);

		priorityDef.setText("75");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(4, 22, 0, 100);
		jPanel1.add(priorityDef, gridBagConstraints);

		priorityDefButton.setText("priority");
		//priorityDefButton.setPreferredSize(new Dimension(69,23));
		priorityDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				priorityDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 16;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(4, 20, 0, 170);
		jPanel1.add(priorityDefButton, gridBagConstraints);

		nolanesDefButton.setText("nolanes");
		//nolanesDefButton.setPreferredSize(new Dimension(69,23));
		nolanesDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				nolanesDefButtonActionPerformed(evt);
			}
		});

		
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 11;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 20, 0, 170);
		jPanel1.add(nolanesDefButton, gridBagConstraints);

		nolanesDef.setText("1");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(0, 22, 0, 100);
		jPanel1.add(nolanesDef, gridBagConstraints);

		parkingCheck.setText("parking");		
		parkingCheck.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent evt) {
				parkingCheckItemStateChanged(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.insets = new Insets(0, 20, 12, 0);
		jPanel1.add(parkingCheck, gridBagConstraints);

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

	class TypeDynamicTableModel extends AbstractTableModel {
	    private String[] columnNames = {"ID","Name","Priority","No Lanes","Speed","Capacity","Parking"};
		Vector vectorTableData = new Vector(10,2);

		public int getColumnCount() {
			return 7;
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
				System.out.println("getValue error! (type)");
				return null;}
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

			String extension = ".type.xml";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			filename=chooser.getSelectedFile().getPath();

			Load l = new Load(filename);
			l.initTypes();
			l.readfile();
			typeData test = l.getTypeData();
			ArrayList list = test.returnData();
			
			for(int i =0;i<list.size();i++){
				typeParameter e = (typeParameter) list.get(i);
				Vector newRow = new Vector();
				newRow.addElement(e.getID()); //ID
				newRow.addElement(e.getName()); //Name
				newRow.addElement(e.getPriority()); //Priority
				newRow.addElement(e.getNolanes()); //No Lanes
				newRow.addElement(e.getSpeed()); //Speed
				newRow.addElement(e.getCapacity()); //Capacity
				newRow.addElement(e.getParkingPossible());
				
				/*
				String parking = e.getParkingPossible();
				if (parking.equals("1")){
					newRow.addElement(new Boolean(true)); //Parking
				}
				else {
					newRow.addElement(new Boolean(false)); //Parking
				}
				*/
				
				mytable.addRow(newRow);
			}
		}

		private void saveFile() {
			typeData m = new typeData();
			typeParameter n;
			String[] temp = {"","","","","","",""};
			for(int i = 0; i<mytable.getRowCount(); i++){
				for (int j=0; j<mytable.getColumnCount(); j++) {				    
					if (j==6){
						//System.out.println(mytable.getValueAt(i,j));
						//if ((Boolean)mytable.getValueAt(i,j) == Boolean.TRUE){							
							temp[j]=(String)mytable.getValueAt(i, j);							
						
					}
					else {					    
						temp[j]=(String)mytable.getValueAt(i, j);
					}
				}
				n = new typeParameter(temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6]);
				m.addTypes(n);
			}
			construct typeconstruction = new construct(m,filename);
		}
		
		private void saveMenuActionPerformed(ActionEvent evt) {
			if (filename == "untitled"){
				String extension = ".type.xml";
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
			String extension = ".type.xml";
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

		private void addRowButtonActionPerformed(ActionEvent evt) {
			Vector newRow = new Vector();
			newRow.addElement(new String("")); //ID
			newRow.addElement(new String("")); //Name
			newRow.addElement(new String("")); //Priority
			newRow.addElement(new String("")); //No Lanes
			newRow.addElement(new String("")); //Speed
			newRow.addElement(new String("")); //Capacity
			//newRow.addElement(new Boolean(false)); //Parking			
			newRow.addElement(new String("")); //Parking
			mytable.addRow(newRow);
		}

		private void removeRowButtonActionPerformed(ActionEvent evt) {
			if (mytable.getRowCount() > 0)
			mytable.removeRow(mytable.getRowCount()-1);
		}

		private void automaticButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<mytable.getRowCount(); i++)
			mytable.setValueAt(new String(""+i),i,0);
		}

		private void quickHelpMenuActionPerformed(ActionEvent evt) {
			JOptionPane.showMessageDialog(this, "Not yet available");
		}

		private void parkingCheckItemStateChanged(ItemEvent evt) {
			String ParkTure = new String("1");
			String ParkFalse = new String("0");			
			if (evt.getStateChange() == ItemEvent.SELECTED) {
				for (int i = 0; i < mytable.getRowCount(); i++)
					mytable.setValueAt(ParkTure,i,6);
					//mytable.setValueAt(new Boolean(true),i,6);					
			}
			else {				
				for (int i = 0; i < mytable.getRowCount(); i++)
					mytable.setValueAt(ParkFalse,i,6);
					//mytable.setValueAt(new Boolean(false),i,6);										
			}
		}

		private void capacityDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(capacityDef.getText(),i,5);
		}

		private void speedDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(speedDef.getText(),i,4);
		}

		private void nolanesDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(nolanesDef.getText(),i,3);
		}

		private void priorityDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(priorityDef.getText(),i,2);
		}

		public static void main(String args[]) {
			new typeEditor();
		}
	}
