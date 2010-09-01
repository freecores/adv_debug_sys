package advancedWatchpointControl;

public interface RegisterObserver {
	public enum updateDirection { GUI_TO_REGS, REGS_TO_GUI }
	
	void notifyRegisterUpdate(updateDirection dir)throws NumberFormatException;
}
