package domains.trench;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class GraphTrenchRF implements RewardFunction {

	int terminalStateID;
	
	public GraphTrenchRF(int terminalStateID) {
		this.terminalStateID = terminalStateID;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		if(this.terminalStateID == GraphDefinedDomain.getNodeId(s)) {
			return 100.0;
		}
		else {
			return -1.0;
		}
	}

}