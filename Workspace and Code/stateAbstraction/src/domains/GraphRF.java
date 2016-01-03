package domains;

import java.util.HashMap;
import java.util.List;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class GraphRF implements RewardFunction {

	private RewardFunction nonGraphRF;
	private HashMap<Integer, State> graphIndexToNonGraphState;
	private HashMap<Integer, Action> graphActionIndexToNonGraphAction;
	private Domain graphDomain;
	private TerminalFunction nonGraphTF;
	
	/**
	 * 
	 * @param graphIndexToNonGraphState
	 * @param graphActionToNonGraphAction
	 * @param nonGraphRF
	 * @param graphDomain
	 */
	public GraphRF(Domain graphDomain, RewardFunction nonGraphRF, TerminalFunction nonGraphTF, HashMap<Integer, State> graphIndexToNonGraphState, HashMap<Integer, Action> graphActionToNonGraphAction) {
		this.nonGraphRF = nonGraphRF;
		this.graphIndexToNonGraphState = graphIndexToNonGraphState;
		this.graphActionIndexToNonGraphAction = graphActionToNonGraphAction;
		this.graphDomain = graphDomain;
		this.nonGraphTF = nonGraphTF;
	}
	
	@Override
	public double reward(State s, GroundedAction a, State sprime) {
		int stateIndex = GraphDefinedDomain.getNodeId(s);
		State nonGraphState = this.graphIndexToNonGraphState.get(stateIndex);
		int sprimeIndex = GraphDefinedDomain.getNodeId(sprime);
		Action nonGraphAction = this.graphActionIndexToNonGraphAction.get(graphDomain.getActions().indexOf(a.action));
		
		// Terminal is non-zero absorbing.
		if (this.nonGraphTF.isTerminal(nonGraphState)) return 0;
		
		//Otherwise just return what the old RF would return.
		return this.nonGraphRF.reward(
				nonGraphState, 
				nonGraphAction.getAssociatedGroundedAction(), 
				this.graphIndexToNonGraphState.get(sprimeIndex));
	}

}