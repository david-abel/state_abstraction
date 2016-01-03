package domains;

import java.util.HashMap;
import java.util.List;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class GraphRF implements RewardFunction {

	private RewardFunction nonGraphRF;
	private HashMap<Integer, State> graphIndexToNonGraphState;
	private HashMap<Action, Action> graphActionToNonGraphAction;
	
	public GraphRF(HashMap<Integer, State> graphIndexToNonGraphState, HashMap<Action, Action> graphActionToNonGraphAction, RewardFunction nonGraphRF) {
		this.nonGraphRF = nonGraphRF;
		this.graphIndexToNonGraphState = graphIndexToNonGraphState;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		int stateIndex = GraphDefinedDomain.getNodeId(s);
		int sprimeIndex = GraphDefinedDomain.getNodeId(sprime);
		
		return this.nonGraphRF.reward(
				this.graphIndexToNonGraphState.get(stateIndex), 
				this.graphActionToNonGraphAction.get(a.action).getAssociatedGroundedAction(), 
				this.graphIndexToNonGraphState.get(sprimeIndex));
	}

}