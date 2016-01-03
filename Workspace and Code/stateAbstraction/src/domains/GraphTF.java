package domains;

import java.util.List;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;

public class GraphTF implements TerminalFunction {

	List<Integer> terminalStateIDs;
	
	public GraphTF(List<Integer> terminalStateIDs) {
		this.terminalStateIDs = terminalStateIDs;
	}
	
	@Override
	public boolean isTerminal(State s) {
		int nodeID = GraphDefinedDomain.getNodeId(s);
		return terminalStateIDs.contains(nodeID);
	}

}