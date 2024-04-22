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

public class manualRoute extends JFrame {

	private String filename = "untitled";

	private VtypeDynamicTableModel vtypetable = new VtypeDynamicTableModel();
	private RouteDynamicTableModel routetable = new RouteDynamicTableModel();
	private VehicleDynamicTableModel vehicletable = new VehicleDynamicTableModel();

	private JTable table1;
	private JTable table2;
	private JTable table3;

	static JFileChooser chooser;

	private JMenu Help;
	private JButton addvehicleButton;
	private JButton addRouteButton;
	private JButton addVehButton;
	private JMenu fileMenu;
	private JLabel jLabel1;
	private JLabel jLabel2;
	private JLabel jLabel3;
	private JLabel jLabel4;
	private JMenuBar jMenuBar2;
	private JPanel jPanel1;
	private JScrollPane jScrollPane1;
	private JScrollPane jScrollPane2;
	private JScrollPane jScrollPane3;

	private JMenuItem newMenu;
	private JMenuItem openMenu;
	private JMenuItem quickHelpMenu;
	private JMenuItem quitMenu;
	private JButton removevehicleButton;
	private JButton removeRouteButton;
	private JButton removeVehButton;
	private JMenuItem saveAsMenu;
	private JMenuItem saveMenu;

	public manualRoute() {
		initComponents();
	}

	private void initComponents() {
		GridBagConstraints gridBagConstraints;

		jPanel1 = new JPanel();
		jScrollPane1 = new JScrollPane();
		jScrollPane2 = new JScrollPane();
		jScrollPane3 = new JScrollPane();

		jLabel1 = new JLabel();
		jLabel2 = new JLabel();
		jLabel3 = new JLabel();
		jLabel4 = new JLabel();

		addVehButton = new JButton();
		removeVehButton = new JButton();

		addRouteButton = new JButton();
		removeRouteButton = new JButton();

		addvehicleButton = new JButton();
		removevehicleButton = new JButton();

		jMenuBar2 = new JMenuBar();
		fileMenu = new JMenu();
		newMenu = new JMenuItem();
		openMenu = new JMenuItem();
		saveMenu = new JMenuItem();
		saveAsMenu = new JMenuItem();
		quitMenu = new JMenuItem();
		Help = new JMenu();
		quickHelpMenu = new JMenuItem();

		if(chooser == null) {
			chooser = new JFileChooser("Route XML File");
		}

		getContentPane().setLayout(new GridBagLayout());
		setTitle("Manual Vehicle Route Editor");

		jPanel1.setLayout(new GridBagLayout());

		jScrollPane1.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		table1 = new JTable(vtypetable);
		jScrollPane1.setViewportView(table1);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 2;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipadx = 200;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.insets = new Insets(6, 20, 0, 23);
		jPanel1.add(jScrollPane1, gridBagConstraints);

		jScrollPane2.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		table2 = new JTable(routetable);
		jScrollPane2.setViewportView(table2);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 6;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipadx = 200;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.insets = new Insets(6, 20, 0, 23);
		jPanel1.add(jScrollPane2, gridBagConstraints);

		jLabel1.setFont(new Font("Tahoma", 1, 18));
		jLabel1.setText("Manual Routes Editor");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 3;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.insets = new Insets(10, 82, 0, 0);
		jPanel1.add(jLabel1, gridBagConstraints);

		jLabel2.setText("Vehicle Types");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 0;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(30, 20, 0, 0);
		jPanel1.add(jLabel2, gridBagConstraints);

		jLabel3.setText("Routes Descriptions");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 5;
		gridBagConstraints.gridwidth = 2;
		gridBagConstraints.insets = new Insets(0, 20, 0, 0);
		jPanel1.add(jLabel3, gridBagConstraints);

		jLabel4.setText("Vehicle Routes Assignments");
		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.gridwidth = 3;
		gridBagConstraints.gridheight = 2;
		gridBagConstraints.insets = new Insets(10, 20, 0, 0);
		jPanel1.add(jLabel4, gridBagConstraints);

		jScrollPane3.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		table3 = new JTable(vehicletable);
		jScrollPane3.setViewportView(table3);

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 0;
		gridBagConstraints.gridy = 10;
		gridBagConstraints.gridwidth = 5;
		gridBagConstraints.fill = GridBagConstraints.BOTH;
		gridBagConstraints.ipadx = 200;
		gridBagConstraints.ipady = 100;
		gridBagConstraints.insets = new Insets(6, 20, 0, 23);
		jPanel1.add(jScrollPane3, gridBagConstraints);

		addVehButton.setText("Add Vehicle");
		//addVehButton.setPreferredSize(new Dimension(105,23));
		addVehButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addVehButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 3;
		gridBagConstraints.ipadx = 23;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(6, 107, 0, 23);
		jPanel1.add(addVehButton, gridBagConstraints);

		removeVehButton.setText("Remove Vehicle");
		//removeVehButton.setPreferredSize(new Dimension(105,23));
		removeVehButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeVehButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 4;
		gridBagConstraints.ipadx = 3;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(0, 107, 0, 23);
		jPanel1.add(removeVehButton, gridBagConstraints);

		addRouteButton.setText("Add Route");
		//addRouteButton.setPreferredSize(new Dimension(105,23));
		addRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 7;
		gridBagConstraints.ipadx = 27;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(6, 107, 0, 23);
		jPanel1.add(addRouteButton, gridBagConstraints);

		removeRouteButton.setText("Remove Route");
		//removeRouteButton.setPreferredSize(new Dimension(105,23));
		removeRouteButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removeRouteButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 8;
		gridBagConstraints.ipadx = 7;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(0, 107, 0, 23);
		jPanel1.add(removeRouteButton, gridBagConstraints);

		addvehicleButton.setText("Add Assignment");
		//addvehicleButton.setPreferredSize(new Dimension(105,23));
		addvehicleButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				addvehicleButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 11;
		gridBagConstraints.ipadx = 21;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(6, 87, 0, 23);
		jPanel1.add(addvehicleButton, gridBagConstraints);

		removevehicleButton.setText("Remove Assignment");
		//removevehicleButton.setPreferredSize(new Dimension(105,23));
		removevehicleButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				removevehicleButtonActionPerformed(evt);
			}
		});

		gridBagConstraints = new GridBagConstraints();
		gridBagConstraints.gridx = 4;
		gridBagConstraints.gridy = 12;
		gridBagConstraints.ipadx = 1;
		gridBagConstraints.ipady = -3;
		gridBagConstraints.insets = new Insets(0, 87, 14, 23);
		jPanel1.add(removevehicleButton, gridBagConstraints);

		getContentPane().add(jPanel1, new GridBagConstraints());

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

		quitMenu.setText("Quit");
		quitMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quitMenuActionPerformed(evt);
			}
		});

		fileMenu.add(quitMenu);

		jMenuBar2.add(fileMenu);

		Help.setText("Help");
		quickHelpMenu.setText("Quick Help");
		quickHelpMenu.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent evt) {
				quickHelpMenuActionPerformed(evt);
			}
		});

		Help.add(quickHelpMenu);
		/*Quick help is disabled for now*/
		//jMenuBar2.add(Help);
		setJMenuBar(jMenuBar2);
		pack();
		setBounds(0, 0, 800, 700);
		setVisible(true);
	}

	class VtypeDynamicTableModel extends AbstractTableModel {
		private String[] columnNames = {"Type ID","Acceleration","Deceleration","Length","Max Speed"};
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
		}

		class RouteDynamicTableModel extends AbstractTableModel {
			private String[] columnNames = {"Route ID","Road References"};
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

			class VehicleDynamicTableModel extends AbstractTableModel {
				private String[] columnNames = {"ID","Vehicle Type","Route","Departure","Period","Numbers"};
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
					while (vtypetable.getRowCount() != 0)
					vtypetable.removeRow(vtypetable.getRowCount()-1);
					while (routetable.getRowCount() != 0)
					routetable.removeRow(routetable.getRowCount()-1);
					while (vehicletable.getRowCount() != 0)
					vehicletable.removeRow(vehicletable.getRowCount()-1);
					filename = "untitled";
				}

				private void openMenuActionPerformed(ActionEvent evt) throws Exception{
					while (vtypetable.getRowCount() != 0)
					vtypetable.removeRow(vtypetable.getRowCount()-1);
					while (routetable.getRowCount() != 0)
					routetable.removeRow(routetable.getRowCount()-1);
					while (vehicletable.getRowCount() != 0)
					vehicletable.removeRow(vehicletable.getRowCount()-1);

					String extension = ".rou.xml";
					chooser.setFileFilter( new JaveFileFilter(extension) );
					int returnVal = chooser.showOpenDialog(this);
					if(returnVal == JFileChooser.APPROVE_OPTION)
					filename=chooser.getSelectedFile().getPath();

					Load l = new Load(filename);
					l.initRoutes();
					l.readfile();
					routeData data = l.getRouteData();
					ArrayList route = data.returnRoute();
					ArrayList vtype = data.returnType();
					ArrayList vehicle = data.returnVehicles();

					for (int i = 0; i < vtype.size(); i++){
						vtypeParameter t = (vtypeParameter) vtype.get(i);
						Vector newRow = new Vector();
						newRow.addElement(t.getID());
						newRow.addElement(t.getAccel());
						newRow.addElement(t.getDecel());
						newRow.addElement(t.getLength());
						newRow.addElement(t.getMaxspeed());
						vtypetable.addRow(newRow);
					}
					for (int i = 0; i < route.size(); i++){
						routeParameter r = (routeParameter) route.get(i);
						Vector newRow = new Vector();
						newRow.addElement(r.getID());
						newRow.addElement(r.getPath());
						routetable.addRow(newRow);
					}
					for (int i = 0; i < vehicle.size(); i++){
						vehicleParameter v = (vehicleParameter) vehicle.get(i);
						Vector newRow = new Vector();
						newRow.addElement(v.getID());
						newRow.addElement(v.getType());
						newRow.addElement(v.getRoute());
						newRow.addElement(v.getDepart());
						newRow.addElement(v.getPeriod());
						newRow.addElement(v.getRepno());
						vehicletable.addRow(newRow);
					}

				}
				
				private void saveFile(){
					routeData m = new routeData();

					vtypeParameter   t;
					routeParameter   r;
					vehicleParameter v;

					String[] temp = {"","","","","",""};

					for(int i = 0; i<vtypetable.getRowCount(); i++){
						for (int j=0; j<vtypetable.getColumnCount(); j++) {
							temp[j]=(String)vtypetable.getValueAt(i, j);
						}
						t = new vtypeParameter(temp[0],temp[1],temp[2],"0.5",temp[3],temp[4]);
						m.addTypes(t);
					}

					for(int i = 0; i<routetable.getRowCount(); i++){
						for (int j=0; j<routetable.getColumnCount(); j++) {
							temp[j]=(String)routetable.getValueAt(i, j);
						}
						r = new routeParameter(temp[0],"x",temp[1]);
						m.addRoutes(r);
					}

					for(int i = 0; i<vehicletable.getRowCount(); i++){
						for (int j=0; j<vehicletable.getColumnCount(); j++) {
							temp[j]=(String)vehicletable.getValueAt(i, j);
						}
						v = new vehicleParameter(temp[0],temp[2],temp[1],temp[3],temp[4],temp[5]);
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

				private void quitMenuActionPerformed(ActionEvent evt) {
					this.dispose();
				}

				private void quickHelpMenuActionPerformed(ActionEvent evt) {
					JOptionPane.showMessageDialog(this, "Not yet available");
				}

				private void removevehicleButtonActionPerformed(ActionEvent evt) {
					if (vehicletable.getRowCount() != 0)
					vehicletable.removeRow(vehicletable.getRowCount()-1);
				}

				private void addvehicleButtonActionPerformed(ActionEvent evt) {
					Vector newRow = new Vector();
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					vehicletable.addRow(newRow);
				}

				private void removeRouteButtonActionPerformed(ActionEvent evt) {
					if (routetable.getRowCount() != 0)
					routetable.removeRow(routetable.getRowCount()-1);
				}

				private void addRouteButtonActionPerformed(ActionEvent evt) {
					Vector newRow = new Vector();
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					routetable.addRow(newRow);
				}

				private void removeVehButtonActionPerformed(ActionEvent evt) {
					if (vtypetable.getRowCount() != 0)
					vtypetable.removeRow(vtypetable.getRowCount()-1);
				}

				private void addVehButtonActionPerformed(ActionEvent evt) {
					Vector newRow = new Vector();
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					newRow.addElement(new String(""));
					vtypetable.addRow(newRow);
				}

				public static void main(String args[]) {
					new manualRoute();
				}
			}
