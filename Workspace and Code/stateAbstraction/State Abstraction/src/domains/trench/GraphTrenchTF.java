package domains.trench;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;

public class GraphTrenchTF implements TerminalFunction {

	int terminalStateID;
	
	public GraphTrenchTF(int terminalStateID) {
		this.terminalStateID = terminalStateID;
	}
	
	@Override
	public boolean isTerminal(State s) {
		if(this.terminalStateID == GraphDefinedDomain.getNodeId(s)) {
			return true;
		}
		else {
			return false;
		}
	}

}