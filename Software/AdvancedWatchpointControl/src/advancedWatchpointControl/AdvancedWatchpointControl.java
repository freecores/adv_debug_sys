package advancedWatchpointControl;

import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.custom.ScrolledComposite;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.RowLayout;



public class AdvancedWatchpointControl {

	private static Display mainDisplay = null;
	private static Shell mainShell = null;
	private static guiServerGroup gServerGroup = null;
	private static guiDCRGroup gDCRGroup = null;
	private static guiCountRegsGroup gCountGroup = null;
	private static guiControlGroup gControlGroup = null;
	private static mainControl mCtrl = null;
	private static ScrolledComposite mainSC = null;
	private static Composite mainComposite = null;
	
	/**
	 * This method initializes mainShell	
	 *
	 */
	private static void createMainShell(Display disp, mainControl mc) {		
		RowLayout mainLayout = new RowLayout();		
		mainLayout.center = true;
		mainLayout.fill = true;
		mainLayout.spacing = 5;
		mainLayout.wrap = false;
		mainLayout.pack = true;
		mainLayout.type = SWT.VERTICAL;
		
		mainShell = new Shell();
		mainShell.setText("Advanced Watchpoint Control");
		mainShell.setLayout(new FillLayout());
		
		mainSC = new ScrolledComposite(mainShell, SWT.H_SCROLL|SWT.V_SCROLL);
		mainComposite = new Composite(mainSC, SWT.NONE);
		mainComposite.setLayout(mainLayout);
		
		gServerGroup = new guiServerGroup(mainComposite, mc);
		gDCRGroup = new guiDCRGroup(mainComposite, mainDisplay, mc);
		gCountGroup = new guiCountRegsGroup(mainComposite, mainDisplay, mc);
		gControlGroup = new guiControlGroup(mainComposite, mc);
		
		mainSC.setContent(mainComposite);
		// Set the minimum size
	    mainSC.setMinSize(770, 950);
	    // Expand both horizontally and vertically
	    mainSC.setExpandHorizontal(true);
	    mainSC.setExpandVertical(true);
	}
	

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		// Create the main control.
		// This also creates the network and RSP subsystems
		mCtrl = new mainControl();
		
		// Create the GUI.  Must be done after main control creation.
		mainDisplay = new Display();
		createMainShell(mainDisplay, mCtrl);
		
		// All ready, show the UI
		mainShell.pack();
		mainShell.open();
		while (!mainShell.isDisposed()) {
		if (!mainDisplay.readAndDispatch()) mainDisplay.sleep();
		}
		mainDisplay.dispose();
	}

}
