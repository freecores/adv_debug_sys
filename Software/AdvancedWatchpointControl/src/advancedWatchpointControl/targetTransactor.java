// This class handles the top-level target transactions.
// Its methods are specific transactions (read register,
// is target running, etc.).  It constructs an appropriate
// transaction object, then gives it to the RSP algorithm
// for processing.


package advancedWatchpointControl;

import java.io.IOException;

public class targetTransactor {

	rspCoder rsp = null;
	
	public targetTransactor(rspCoder r) {
		rsp = r;
	}
	
	// Succeeds or throws an IOException.
	public void writeRegister(targetDebugRegisterSet.regType reg, long val) throws IOException {
		WriteRegisterTransaction xact = new WriteRegisterTransaction(reg, val);
		rsp.Transact(xact);
	}
	
	// Returns a valid value or throws an IOException.
	public long readRegister(targetDebugRegisterSet.regType reg) throws IOException {
		long ret;
		ReadRegisterTransaction xact = new ReadRegisterTransaction(reg);
		rsp.Transact(xact);
		ret = xact.getDataValueRead();
		return ret;
	}
	
	// Returns a valid boolean indicator or throws an IOException.
	public boolean isTargetRunning() throws IOException {
		boolean ret;
		TargetRunningTransaction xact = new TargetRunningTransaction();
		rsp.Transact(xact);
		ret = xact.getIsTargetRunning();
		return ret;
	}
	
}
