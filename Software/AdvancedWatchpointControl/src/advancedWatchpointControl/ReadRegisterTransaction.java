package advancedWatchpointControl;

public class ReadRegisterTransaction implements TargetTransaction {

	private String packetString = null;
	private long dataValueRead = 0;
	
	public ReadRegisterTransaction(targetDebugRegisterSet.regType reg) {
		packetString = new String("p"); // 'p' is read one register
		int regAddr = targetDebugRegisterSet.getRegisterAddress(reg);
		packetString += Integer.toHexString(regAddr);
	}
	
	@Override
	public String getDataToSend() {
		return packetString;
	}

	@Override
	public boolean receivePacket(String pkt) {

		// A register read response has no leading header / char...
		// so just parse the number.
		long val;
		try {
			val = Long.parseLong(pkt, 16);  // data comes back as a hex string
		} catch (Exception e) {
			// TODO logMessageGUI("Got invalid read data (size " + pkt.length() + "): " + pkt + ": " + e);
			dataValueRead = 0;
			return false;
		}
		
		dataValueRead = val;
		return true;
	}

	public long getDataValueRead() {
		return dataValueRead;
	}
	
}
