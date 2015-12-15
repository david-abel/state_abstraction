package domains;

import java.util.List;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class GraphRF implements RewardFunction {

	List<Integer> terminalStateIDs;
	
	public GraphRF(List<Integer> terminalStateIDs) {
		this.terminalStateIDs = terminalStateIDs;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		if(terminalStateIDs.contains(GraphDefinedDomain.getNodeId(sprime))) {
			return 100.0;
		}
		else {
			return -1.0;
		}
	}

}