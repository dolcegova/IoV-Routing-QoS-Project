package com.move;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.commonData.tripData;
import com.move.commonData.tripParameter;

import java.awt.event.*;
import java.awt.*;

public class tripEditor extends JFrame {

	private String filename = "untitled";
	private TripDynamicTableModel mytable = new TripDynamicTableModel();
	private JTable table;
	static JFileChooser chooser;

	private JButton addRowButton;
	private JButton departureDefButton;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JMenuBar jMenuBar;
	private JPanel jPanel1;
	private JScrollPane jScrollPane1;
	private JSeparator jSeparator1;

	private JTextField departureDef;
	private JMenuItem newMenu;
	private JMenuItem openMenu;
	private JTextField periodDef;
	private JButton periodDefButton;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removeRowButton;
	private JTextField repnoDef;
	private JButton repnoDefButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;

	public tripEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jScrollPane1 = new JScrollPane();

		departureDefButton = new JButton();
		addRowButton = new JButton();
		removeRowButton = new JButton();
		jLabel1 = new JLabel();
		departureDef = new JTextField();
		jLabel2 = new JLabel();
		periodDefButton = new JButton();
		repnoDefButton = new JButton();
		periodDef = new JTextField();
		repnoDef = new JTextField();
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
		setTitle("Vehicle Trip Definitions Editor");

		table = new JTable(mytable);
		jScrollPane1.setViewportView(table);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 1;
		gridBagConstraints.gridwidth = 4;
		gridBagConstraints.ipadx = 37;
		gridBagConstraints.ipady = -209;
		gridBagConstraints.insets = new Insets(14, 10, 0, 16);
		jPanel1.add(jScrollPane1, gridBagConstraints);

		departureDefButton.setText("departure");
		//departureDefButton.setPreferredSize(new Dimension(69,23));
		departureDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				departureDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 10, 0, 70);
		jPanel1.add(departureDefButton, gridBagConstraints);

		addRowButton.setText("Add Trip");
		//addRowButton.setPreferredSize(new Dimension(79,23));
		addRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.ipadx = 36;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(20, 180, 10, 0);
		jPanel1.add(addRowButton, gridBagConstraints);

		removeRowButton.setText("Remove Trip");
		//removeRowButton.setPreferredSize(new Dimension(79,23));
		removeRowButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeRowButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 2;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 12;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 180, 0, 0);
		jPanel1.add(removeRowButton, gridBagConstraints);

		jLabel1.setText("Set Defaults");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.insets = new Insets(10, 10, 4, 0);
		jPanel1.add(jLabel1, gridBagConstraints);

		departureDef.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(0, 21, 0, 0);
		jPanel1.add(departureDef, gridBagConstraints);

		jLabel2.setText("Vehicle Trips Definitions Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.insets = new Insets(10, 10, 0, 287);
		jPanel1.add(jLabel2, gridBagConstraints);

		periodDefButton.setText("period");
		//periodDefButton.setPreferredSize(new Dimension(69,23));
		periodDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				periodDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 20;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 10, 0, 70);
		jPanel1.add(periodDefButton, gridBagConstraints);

		repnoDefButton.setText("repno");
		//repnoDefButton.setPreferredSize(new Dimension(69,23));
		repnoDefButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				repnoDefButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.ipadx = 23;
		gridBagConstraints.ipady = -6;
		gridBagConstraints.insets = new Insets(0, 10, 42, 70);
		jPanel1.add(repnoDefButton, gridBagConstraints);

		periodDef.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(0, 21, 0, 0);
		jPanel1.add(periodDef, gridBagConstraints);

		repnoDef.setText("0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.ipadx = 59;
		gridBagConstraints.insets = new Insets(0, 21, 42, 0);
		jPanel1.add(repnoDef, gridBagConstraints);

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

	class TripDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"ID","Departure","From Edge","To Edge","Type","Period","No Vehicles"};
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

			String extension = ".trips.xml";
			chooser.setFileFilter( new JaveFileFilter(extension) );
			int returnVal = chooser.showOpenDialog(this);
			if(returnVal == JFileChooser.APPROVE_OPTION)
			filename=chooser.getSelectedFile().getPath();

			Load l = new Load(filename);
			l.initTrips();
			l.readfile();
			tripData test = l.getTripData();
			ArrayList list = test.returnData();

			for(int i =0;i<list.size();i++){
				tripParameter e = (tripParameter) list.get(i);
				Vector newRow = new Vector();

				newRow.addElement(e.getID());
				newRow.addElement(e.getDepart());
				newRow.addElement(e.getFrom());
				newRow.addElement(e.getTo());
				newRow.addElement(e.getType());
				newRow.addElement(e.getPeriod());
				newRow.addElement(e.getRepno());
				mytable.addRow(newRow);
			}
		}

		private void saveFile() {
			tripData m = new tripData();
			tripParameter n;
			String[] temp = {"","","","","","",""};
			for(int i = 0; i<mytable.getRowCount(); i++){
				for (int j=0; j<mytable.getColumnCount(); j++) {
					temp[j]=(String)mytable.getValueAt(i, j);
					}
				n = new tripParameter(temp[0],temp[1],temp[2],temp[3],temp[5],temp[4],temp[6]);	
				m.addTrips(n);
				}
			construct tripconstruction = new construct(m,filename);
		}
		
		private void saveMenuActionPerformed(ActionEvent evt) {			
			if (filename == "untitled"){
				String extension = ".trips.xml";
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
			String extension = ".trips.xml";
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
			newRow.addElement(new String("")); //Departure
			newRow.addElement(new String("")); //From
			newRow.addElement(new String("")); //To
			newRow.addElement(new String("")); //Type
			newRow.addElement(new String("")); //Period
			newRow.addElement(new String("")); //Vehicles
			mytable.addRow(newRow);
		}

		private void removeRowButtonActionPerformed(ActionEvent evt) {
			if (mytable.getRowCount() > 0)
			mytable.removeRow(mytable.getRowCount()-1);
		}

		private void automaticButtonActionPerformed(ActionEvent evt) {
			for (int i= 0; i<mytable.getRowCount(); i++)
			mytable.setValueAt(new String("trip"+i),i,0);
		}

		private void quickHelpMenuActionPerformed(ActionEvent evt) {
			JOptionPane.showMessageDialog(this, "Not yet available");
		}

		private void repnoDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(repnoDef.getText(),i,6);
		}

		private void periodDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(periodDef.getText(),i,5);
		}

		private void departureDefButtonActionPerformed(ActionEvent evt) {
			for (int i = 0; i < mytable.getRowCount(); i++)
			mytable.setValueAt(departureDef.getText(),i,1);
		}

		public static void main(String args[]) {
			new tripEditor();
		}

	}
