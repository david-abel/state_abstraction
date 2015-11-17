package stateAbstractor;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.states.State;

public class PhiModel extends StateAbstractor {

	@Override
	// TODO: implement. It is currently just an example of collapsing just the first two indexed graph defined states together.
	protected boolean phi(State s1, State s2) {
		return (GraphDefinedDomain.getNodeId(s1) == 0 && GraphDefinedDomain.getNodeId(s2) == 1)
				||
				(GraphDefinedDomain.getNodeId(s1) == 1 && GraphDefinedDomain.getNodeId(s2) == 0);
	}

}
