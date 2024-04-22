package com.move;
import javax.swing.*;

import java.util.*;

import javax.swing.table.*;

import com.move.commonData.routeData;
import com.move.commonData.routeParameter;
import com.move.commonData.vehicleParameter;
import com.move.commonData.vtypeParameter;

import java.awt.*;
import java.awt.event.*;

/*
Creator: Feliz Kristianto Karnadi and Zhi Hai Mo
UNSW - Sydney
*/

public class busTimetableEditor extends JFrame {

	private String filename = "untitled";
	static JFileChooser chooser;
	private TimetableDynamicTableModel timetable = new TimetableDynamicTableModel();
	private RouteDynamicTableModel routetable = new RouteDynamicTableModel();
	private BusRoutes routeList = new BusRoutes();

	private JTextField accel;
	private JButton addNewRouteButton;
	private JButton addRoutePathButton;
	private JButton addTimetableButton;
	private JTextField decel;
	private JMenu fileMenu;
	private JMenu helpMenu;
	private JLabel jLabel1;
	private JLabel jLabel10;
	private JLabel jLabel11;
	private JLabel jLabel13;
	private JLabel jLabel14;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JLabel jLabel5;
	private JLabel jLabel6;
	private JLabel jLabel8;
	private JLabel jLabel9;
	private JMenuBar jMenuBar;
	private JScrollPane jScrollPane1;
	private JScrollPane jScrollPane3;
	private JSeparator jSeparator1;
	private JTable jTable1;
	private JTable jTable3;
	private JTextField maxSpeed;
	private JMenuItem newMenu;
	private JTextField newRoute;
	private JMenuItem openMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removeRoutePathButton;
	private JButton removeSelectedRouteButton;
	private JButton removeTimetableButton;
	private JComboBox routeCombo;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;

	public busTimetableEditor() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jScrollPane3 = new JScrollPane();
		jTable3 = new JTable();
		addTimetableButton = new JButton();
		removeTimetableButton = new JButton();
		jLabel3 = new JLabel();
		routeCombo = new JComboBox();
		jScrollPane1 = new JScrollPane();
		jTable1 = new JTable();
		addRoutePathButton = new JButton();
		removeRoutePathButton = new JButton();
		newRoute = new JTextField();
		addNewRouteButton = new JButton();
		jLabel1 = new JLabel();
		removeSelectedRouteButton = new JButton();
		jLabel2 = new JLabel();
		jLabel4 = new JLabel();
		jLabel5 = new JLabel();
		jLabel6 = new JLabel();
		maxSpeed = new JTextField();
		jLabel8 = new JLabel();
		jLabel9 = new JLabel();
		jLabel10 = new JLabel();
		accel = new JTextField();
		decel = new JTextField();
		jLabel11 = new JLabel();
		jLabel13 = new JLabel();
		jLabel14 = new JLabel();
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

		getContentPane().setLayout(new GridBagLayout());

		setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		setTitle("Bus Timetable Editor");

		if(chooser == null) {
			chooser = new JFileChooser("Route XML File");
		}
		jScrollPane3.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		jTable3 = new JTable(timetable);
		jScrollPane3.setViewportView(jTable3);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 34;
		gridBagConstraints.gridwidth = 29;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.weightx = 1.0;
		gridBagConstraints.weighty = 1.0;
		gridBagConstraints.insets = new Insets(0, 20, 0, 20);
		getContentPane().add(jScrollPane3, gridBagConstraints);

		addTimetableButton.setText("Add Timetable");
		addTimetableButton.setMaximumSize(new Dimension(121, 23));
		addTimetableButton.setMinimumSize(new Dimension(121, 23));
		addTimetableButton.setPreferredSize(new Dimension(121, 23));
		addTimetableButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addTimetableButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 23;
		gridBagConstraints.gridy = 36;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.ipadx = 15;
		gridBagConstraints.anchor = GridBagConstraints.EAST;
		gridBagConstraints.insets = new Insets(10, 0, 0, 20);
		getContentPane().add(addTimetableButton, gridBagConstraints);

		removeTimetableButton.setText("Remove Timetable");
		removeTimetableButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeTimetableButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 23;
		gridBagConstraints.gridy = 37;
		gridBagConstraints.gridwidth = 6;
		gridBagConstraints.anchor = GridBagConstraints.EAST;
		gridBagConstraints.insets = new Insets(10, 0, 10, 20);
		getContentPane().add(removeTimetableButton, gridBagConstraints);

		jLabel3.setFont(new Font("Tahoma", 1, 18));
		jLabel3.setText("Bus Timetable Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 19;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.ipadx = 300;
		gridBagConstraints.insets = new Insets(10, 10, 20, 0);
		getContentPane().add(jLabel3, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 100;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(routeCombo, gridBagConstraints);

		/*route Combo Box is not editable*/
		routeCombo.setEditable(false);
		routeCombo.addItem("aaaaaaaaaaaaaaaaaaaaaaaa");
		
		routeCombo.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				routeComboActionPerformed(evt);
			}
		});

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		jTable1 = new JTable(routetable);
		jScrollPane1.setViewportView(jTable1);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 20;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.gridheight = 27;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.weightx = 1.0;
		gridBagConstraints.weighty = 1.0;
		gridBagConstraints.insets = new Insets(0, 20, 0, 20);
		getContentPane().add(jScrollPane1, gridBagConstraints);

		addRoutePathButton.setText("Add Path");
		addRoutePathButton.setMaximumSize(new Dimension(111, 23));
		addRoutePathButton.setMinimumSize(new Dimension(111, 23));
		addRoutePathButton.setPreferredSize(new Dimension(111, 23));
		addRoutePathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addRoutePathButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 20;
		gridBagConstraints.gridy = 31;
		gridBagConstraints.ipadx = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.EAST;
		gridBagConstraints.insets = new Insets(10, 0, 0, 20);
		getContentPane().add(addRoutePathButton, gridBagConstraints);

		removeRoutePathButton.setText("Remove Path");
		removeRoutePathButton.setMaximumSize(new Dimension(111, 23));
		removeRoutePathButton.setMinimumSize(new Dimension(111, 23));
		removeRoutePathButton.setOpaque(false);
		removeRoutePathButton.setPreferredSize(new Dimension(111, 23));
		removeRoutePathButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeRoutePathButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 20;
		gridBagConstraints.gridy = 32;
		gridBagConstraints.ipadx = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.EAST;
		gridBagConstraints.insets = new Insets(10, 0, 0, 20);
		getContentPane().add(removeRoutePathButton, gridBagConstraints);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipadx = 180;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(newRoute, gridBagConstraints);

		addNewRouteButton.setText("Add");
		addNewRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addNewRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 1;
		gridBagConstraints.gridy = 26;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		getContentPane().add(addNewRouteButton, gridBagConstraints);

		jLabel1.setText("Insert new route");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		getContentPane().add(jLabel1, gridBagConstraints);

		removeSelectedRouteButton.setText("Remove");
		removeSelectedRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeSelectedRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 29;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(6, 20, 0, 0);
		getContentPane().add(removeSelectedRouteButton, gridBagConstraints);

		jLabel2.setText("Remove selected route");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 28;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(10, 20, 0, 0);
		getContentPane().add(jLabel2, gridBagConstraints);

		jLabel4.setFont(new Font("Tahoma", 1, 11));
		jLabel4.setText("Select Bus Route");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 20, 10, 0);
		getContentPane().add(jLabel4, gridBagConstraints);

		jLabel5.setText("maximum speed");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 24;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 5, 0, 0);
		getContentPane().add(jLabel5, gridBagConstraints);

		jLabel6.setFont(new Font("Tahoma", 1, 11));
		jLabel6.setText("Bus Timetable");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 33;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(7, 20, 0, 0);
		getContentPane().add(jLabel6, gridBagConstraints);

		maxSpeed.setText("70.0");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 25;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 8, 0, 0);
		getContentPane().add(maxSpeed, gridBagConstraints);

		jLabel8.setText("acceleration");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 24;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 5, 0, 0);
		getContentPane().add(jLabel8, gridBagConstraints);

		jLabel9.setFont(new Font("Tahoma", 1, 11));
		jLabel9.setText("Bus Characteristic");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 24;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 5, 10, 0);
		getContentPane().add(jLabel9, gridBagConstraints);

		jLabel10.setText("decceleration");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 24;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 5, 0, 0);
		getContentPane().add(jLabel10, gridBagConstraints);

		accel.setText("0.8");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 25;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 8, 0, 0);
		getContentPane().add(accel, gridBagConstraints);

		decel.setText("4.5");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 25;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.ipadx = 39;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 8, 0, 0);
		getContentPane().add(decel, gridBagConstraints);

		jLabel11.setText("m/s");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 26;
		gridBagConstraints.gridy = 14;
		gridBagConstraints.ipadx = 10;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel11, gridBagConstraints);

		jLabel13.setText("m/s");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 26;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 10;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel13, gridBagConstraints);

		jLabel14.setText("m/s");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 26;
		gridBagConstraints.gridy = 24;
		gridBagConstraints.ipadx = 10;
		gridBagConstraints.ipady = 6;
		gridBagConstraints.anchor = GridBagConstraints.NORTHWEST;
		gridBagConstraints.insets = new Insets(0, 10, 0, 0);
		getContentPane().add(jLabel14, gridBagConstraints);

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

		setBounds(0,0,700,600);
routeCombo.removeItem("aaaaaaaaaaaaaaaaaaaaaaaa");
		//TableColumn routeColumn = jTable3.getColumnModel().getColumn(1);
		//routeColumn.setCellEditor(new DefaultCellEditor(routeCombo));
		setVisible(true);
	}

	class TimetableDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"timetable id","route","departure (hh:mm)","last departure (hh:mm)","period (mins)"};

		Vector vectorTableData = new Vector(10,2);

		public int getColumnCount() {
			return 5;
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

			public void clearTable() {
				int i = getRowCount();
				while (i > 0){
					removeRow(i-1);
					i--;
				}
			}
		}
		class BusRoutes {
			Vector vectorData = new Vector(10,2);

			public void addData(String target){
				Vector newVector = new Vector(10,2);
				newVector.addElement(target);
				Vector newTable = new Vector(10,2);
				newVector.addElement(newTable);
				vectorData.addElement(newVector);
			}

			public void removeData(String target){
				for (int i = 0; i < vectorData.size(); i++){
					Vector temp = (Vector)vectorData.elementAt(i);
					if (((String)temp.elementAt(0)).equals(target)){
						vectorData.removeElementAt(i);
					}
				}
			}
			public Vector getRouteTable(String target){
				for (int i = 0; i <= vectorData.size(); i++){
					Vector temp = (Vector)vectorData.elementAt(i);
					if (((String)temp.elementAt(0)).equals(target)){
						return (Vector)temp.elementAt(1);
					}
				}
				return null;
			}
			public void addPath(String ID, String path){

				String[] temp = path.split(" ");
				Vector newVector = new Vector(10,2);
				newVector.addElement(ID);
				Vector newTable = new Vector(10,2);
				for (int i = 0; i < temp.length; i++){
					Vector newPath = new Vector();
					newPath.addElement(temp[i]);
					newTable.addElement(newPath);
				}
				newVector.addElement(newTable);
				vectorData.addElement(newVector);
			}

			public int size(){
				return vectorData.size();
			}

			public Vector elementAt(int index){
				return (Vector)vectorData.elementAt(index);
			}

			public void clear(){
				while (vectorData.size() > 0){
					vectorData.removeElementAt(vectorData.size()-1);
				}
			}

		}
		class RouteDynamicTableModel extends AbstractTableModel {
			private String[] columnNames = {"route path"};
			private int flag = 0;
			Vector vectorTableData = new Vector(10,2);

			public int getColumnCount() {
				return 1;
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
					Vector temp = routeList.getRouteTable((String)routeCombo.getSelectedItem());
					((Vector)temp.elementAt(row)).setElementAt(value,col);

				}
				public void addRow(Vector someVector) {
					vectorTableData.addElement(someVector);
					Vector temp = routeList.getRouteTable((String)routeCombo.getSelectedItem());
					temp.addElement(someVector);
					fireTableRowsInserted(vectorTableData.size() - 1, vectorTableData.size() -1);
				}
				public void removeRow(int row) {
					vectorTableData.removeElementAt(row);
					Vector temp = routeList.getRouteTable((String)routeCombo.getSelectedItem());
					temp.removeElementAt(row);
					fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
				}
				public void clearTable() {
					int i = getRowCount();
					while (i > 0){
						vectorTableData.removeElementAt(i-1);
						fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
						i--;
					}
				}
				public void setTable(String target) {
					clearTable();
					Vector temp = routeList.getRouteTable(target);
					int i = temp.size();
					if (temp.size() > 0){
						for (int x = 0; x < temp.size(); x++){
							vectorTableData.addElement((Vector)temp.elementAt(x));
							fireTableRowsDeleted(vectorTableData.size(), vectorTableData.size());
						}
					}

				}
			}

			private void addNewRouteButtonActionPerformed(ActionEvent evt) {
				routeList.addData(newRoute.getText());
				routeCombo.addItem(newRoute.getText());
				routeCombo.setSelectedItem(newRoute.getText());
			}

			private void removeSelectedRouteButtonActionPerformed(ActionEvent evt) {
				if (routeCombo.getItemCount() > 0){
					routeCombo.removeItem(routeCombo.getSelectedItem());
					routeList.removeData(newRoute.getText());
				}
			}

			private void routeComboActionPerformed(ActionEvent evt) {
				int i = routeCombo.getItemCount();
				if (i != 0){
					JComboBox temp = (JComboBox)evt.getSource();
					String target = (String)temp.getSelectedItem();
					if (routeList.size() != 0)
					routetable.setTable(target);

				}
			}

			private void quickHelpMenuActionPerformed(ActionEvent evt) {
				JOptionPane.showMessageDialog(this, "Not yet available");
			}

			private void quitMenuActionPerformed(ActionEvent evt) {
				this.dispose();
			}

			private void saveFile() {
				routeData m = new routeData();

				vtypeParameter   t;
				routeParameter   r;
				vehicleParameter v;

				//System.out.println("Vehicle Type : standard, length : 5");
				//System.out.println("Accel: " + accel.getText() + " Decel: " + decel.getText() + " Maxspeed : " + maxSpeed.getText());
				//System.out.println();

				t = new vtypeParameter("default",accel.getText(),decel.getText(),"0.5","5",maxSpeed.getText());
				m.addTypes(t);

				//System.out.println("Route definitions");
				//System.out.println("Size of route is " + routeList.size());

				String path = "";
				for (int i = 0; i < routeList.size(); i++){
					Vector temp = routeList.elementAt(i);
					path = "";
					//System.out.print("Route ID: " + (String)temp.elementAt(0) + " path is");
					Vector tabletemp = (Vector)temp.elementAt(1);
					for (int j = 0; j < tabletemp.size(); j++) {
						Vector temp2 = (Vector)tabletemp.elementAt(j);
						//System.out.print(" " + (String)(temp2.elementAt(0)));
						path = path + (String)(temp2.elementAt(0));
						if (j != tabletemp.size()-1)
						path = path + " ";
					}


					r = new routeParameter((String)temp.elementAt(0),"x",path);
					m.addRoutes(r);
				}

				//System.out.println("Bus timetable");
				String temp[] = {"","","","",""};

				for(int i = 0; i<timetable.getRowCount(); i++){
					for (int j=0; j<timetable.getColumnCount(); j++) {
						//if (j == 1){
						// temp[j] = (String)(((JComboBox) (timetable.getValueAt(i,j))).getSelectedItem());
						//}
						//else {
						temp[j]=(String)timetable.getValueAt(i, j);
						//}
					}

					//System.out.print("timetable id: " + temp[0]);
					//System.out.print(" vehicle type: default");
					//System.out.print(" route: " + temp[1]);
					//System.out.print(" departure: ");

					int hour10 = temp[2].charAt(0) - 48;
					int hour1  = temp[2].charAt(1) - 48;
					int mins10 = temp[2].charAt(3) - 48;
					int mins1  = temp[2].charAt(4) - 48;

					int depart = (hour10 * 10 + hour1) * 360 + (mins10 * 10 + mins1) * 6;
					//System.out.print("" + depart);

					int ehour10 = temp[3].charAt(0) - 48;
					int ehour1  = temp[3].charAt(1) - 48;
					int emins10 = temp[3].charAt(3) - 48;
					int emins1  = temp[3].charAt(4) - 48;

					int end = (ehour10 * 10 + ehour1) * 360 + (emins10 * 10 + emins1) * 6;

					//System.out.print(" last depart: " + end);
					int mins = 0;
					int mult = 1;
					for (int k = temp[4].length()-1; k >= 0; k--){
						int a = temp[4].charAt(k) - 48;
						mins = mins + (a * mult);
						mult = mult * 10;
					}
					//System.out.print(" period min: " + mins);
					int period = mins * 6;
					//System.out.print(" period sec: " + period);

					int vehno = (end - depart) / period;
					//System.out.print(" vehicle no: " + vehno + "\n");

					v = new vehicleParameter(temp[0],temp[1],"default",("" + depart),("" + period),("" + vehno));
					m.addVehicles(v);
				}
				construct routeconstruction = new construct(m,filename);
			}
			
			private void saveMenuActionPerformed(ActionEvent evt) {
				if (filename == "untitled"){
					String extension = ".rou.xml";
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
				String extension = ".rou.xml";
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

			private void openMenuActionPerformed(ActionEvent evt) throws Exception{
				timetable.clearTable();
				routetable.clearTable();
				routeList.clear();
				routeCombo.removeAllItems();

				String extension = ".rou.xml";
				chooser.setFileFilter( new JaveFileFilter(extension) );
				int returnVal = chooser.showOpenDialog(this);
				if(returnVal == JFileChooser.APPROVE_OPTION)
				filename = chooser.getSelectedFile().getPath();

				Load l = new Load(filename);
				l.initRoutes();
				l.readfile();

				routeData data = l.getRouteData();
				ArrayList route = data.returnRoute();
				ArrayList vtype = data.returnType();
				ArrayList vehicle = data.returnVehicles();

				for (int i = 0; i < vtype.size(); i++){
					vtypeParameter t = (vtypeParameter) vtype.get(i);
					accel.setText(t.getAccel());
					decel.setText(t.getDecel());
					maxSpeed.setText(t.getMaxspeed());
				}

				for (int i = 0; i < route.size(); i++){
					routeParameter r = (routeParameter) route.get(i);

					routeCombo.addItem(r.getID());
					routeList.addPath(r.getID(),r.getPath());
					routeCombo.setSelectedItem(r.getID());
				}


				String sdepart = "";
				String speriod = "";
				String svehno = "";
				for (int i = 0; i < vehicle.size(); i++){
					vehicleParameter v = (vehicleParameter) vehicle.get(i);

					Vector newRow = new Vector();
					newRow.addElement(v.getID());

					//JComboBox temp = new JComboBox();
					//for (int j=0; i < routeCombo.getItemCount(); i++){
					//	temp.addItem(routeCombo.getItemAt(i));
					//	temp.setSelectedItem(v.getRoute());
					//}
					//newRow.addElement(temp);

					newRow.addElement(v.getRoute());

					sdepart = v.getDepart();
					int depart = 0;
					int mult = 1;
					for (int j = sdepart.length()-1; j >=0 ; j--){
						depart = depart + ((int)sdepart.charAt(j) - 48) * mult;
						mult = mult*10;
					}
					int origdepart = depart;
					int hour = depart / 360;
					depart = depart - (hour * 360);
					int mins = depart / 6;

					String departurestr = "";
					if (hour <10)
						departurestr = "0" + hour;
					else if (hour == 0)
						departurestr = "00";
					else
						departurestr = departurestr + hour;
					departurestr = departurestr + ":";
					if (mins <10)
						departurestr = departurestr + "0" + mins;
					else if (hour == 0)
						departurestr = departurestr + "00";
					else
						departurestr = departurestr + mins;
					
					newRow.addElement(departurestr);

					speriod = v.getPeriod();
					int period = 0;
					mult = 1;
					for (int j = speriod.length()-1; j >=0 ; j--){
						period = period + ((int)speriod.charAt(j) - 48) * mult;
						mult = mult * 10;
					}

					svehno = v.getRepno();
					int vehno = 0;
					mult = 1;
					for (int j = svehno.length()-1; j >=0 ; j--){
						vehno = vehno + ((int)svehno.charAt(j) - 48) * mult;
						mult = mult * 10;
					}

					int end = origdepart + (period * vehno);

					int ehour = end /360;
					end = end - (ehour * 360);
					int emins = end /6;

					int periodmins = period / 6;

					String ldeparturestr = "";
					if (ehour <10)
						ldeparturestr = "0" + ehour;
					else if (ehour == 0)
						ldeparturestr = "00";
					else
						ldeparturestr = ldeparturestr + ehour;
					ldeparturestr = ldeparturestr + ":";

					if (emins <10)
						ldeparturestr = ldeparturestr + "0" + emins;
					else if (ehour == 0)
						ldeparturestr = ldeparturestr + "00";
					else
						ldeparturestr = ldeparturestr + emins;
						
					newRow.addElement(ldeparturestr);
					newRow.addElement("" + periodmins);

					timetable.addRow(newRow);
				}
			}

			private void newMenuActionPerformed(ActionEvent evt) {
				timetable.clearTable();
				routetable.clearTable();
				routeList.clear();
				routeCombo.removeAllItems();
				filename = "untitled";
			}



			private void addTimetableButtonActionPerformed(ActionEvent evt) {
				Vector newRow = new Vector();
				newRow.addElement(new String(""));

				//JComboBox temp = new JComboBox();
				//if (routeCombo.getItemCount() >0){
				//for (int i=0; i < routeCombo.getItemCount(); i++){
				//	temp.addItem(routeCombo.getItemAt(i));
				//}
				//}
				//TableColumn routeColumn = jTable3.getColumnModel().getColumn(1);
				//routeColumn.setCellEditor(new DefaultCellEditor(temp));
				//newRow.addElement(temp);
				newRow.addElement(new String(""));
				newRow.addElement(new String(""));
				newRow.addElement(new String(""));
				newRow.addElement(new String("5"));
				timetable.addRow(newRow);
			}

			private void removeTimetableButtonActionPerformed(ActionEvent evt) {
				if (timetable.getRowCount() != 0)
				timetable.removeRow(timetable.getRowCount()-1);
			}


			private void addRoutePathButtonActionPerformed(ActionEvent evt) {
				if (routeCombo.getItemCount() > 0){
					Vector newRow = new Vector();
					newRow.addElement(new String(""));
					routetable.addRow(newRow);
				}
			}

			private void removeRoutePathButtonActionPerformed(ActionEvent evt) {
				if (routeCombo.getItemCount() > 0){
					if (routetable.getRowCount() != 0)
					routetable.removeRow(routetable.getRowCount()-1);
				}
			}

			public static void main(String args[]) {
				EventQueue.invokeLater(new Runnable() {
					public void run() {
						new busTimetableEditor();
					}
				});
			}



		}
