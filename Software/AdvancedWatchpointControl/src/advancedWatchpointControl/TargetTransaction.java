package advancedWatchpointControl;

public interface TargetTransaction {
	
	// The RSP (or other encoding / sending algorithm) can
	// call this method to get the pre-formatted packet which should be
	// encoded and sent
	public String getDataToSend();
	
	// This should be called by the receiving portion of the RSP
	// algorithm each time a new character is received from the
	// network and decoded.  It returns true so long as it wants
	// another character.  It 
	public boolean receivePacket(String pkt);
	
}
