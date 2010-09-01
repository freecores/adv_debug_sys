package advancedWatchpointControl;


public class WriteRegisterTransaction implements TargetTransaction {

		private String packetString = null;
	
	public WriteRegisterTransaction(targetDebugRegisterSet.regType reg, long val) {
		packetString = new String("P"); // 'P' is write one register
		int regAddr = targetDebugRegisterSet.getRegisterAddress(reg);
		
		packetString += Integer.toHexString(regAddr);
		packetString += "=";
		
		String valueStr = Long.toHexString(val);
		
		// There must be 8 bytes of 'value'
		if(valueStr.length() > 8) {
			// Use the last 8 bytes, the first 8 may just be a sign extension
			valueStr = valueStr.substring(valueStr.length() - 8, valueStr.length());
		}
		
		int padsize = 8 - valueStr.length();
		for(int i = 0; i < padsize; i++) {
			packetString += '0';
		}
		
		packetString += valueStr;
	}
	
	@Override
	public String getDataToSend() {
		return packetString;
	}

	@Override
	public boolean receivePacket(String pkt) {

		// Only one valid response from a register write: "OK"
		if(pkt.charAt(0) == 'O' && pkt.charAt(1) == 'K') {
			return true;
		}		
		
		return false;
	}

}
