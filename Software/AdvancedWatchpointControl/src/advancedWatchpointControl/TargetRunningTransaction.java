package advancedWatchpointControl;


public class TargetRunningTransaction implements TargetTransaction {

	private boolean isTargetRunning = false;
	
	public TargetRunningTransaction() {
	}
	
	@Override
	public String getDataToSend() {
		return new String("?");
	}

	@Override
	public boolean receivePacket(String pkt) {

		// 'S##' means it's stopped, 'R' means it's running,
		// anything else is an error.
		if(pkt.charAt(0) == 'R') {
			// target is running, disallow accesses
			isTargetRunning = true;
		}
		else if(pkt.charAt(0) == 'S') {
			// We got a stop packet 
			isTargetRunning = false;
		}
		else {
			return false;
		}
		
		return true;
	}

	public boolean getIsTargetRunning() {
		return isTargetRunning;
	}
	
}
