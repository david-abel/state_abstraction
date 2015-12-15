package domains.trench;

import java.util.ArrayList;
import java.util.List;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class TrenchDomainToGraphDomain {

	TrenchDomainGenerator trenchGen;
	Domain oldDomain;
	public List<Integer> goalStateIDs;
	public int initStateID;
	
	public TrenchDomainToGraphDomain(TrenchDomainGenerator tg) {
		this.trenchGen = tg;
		this.goalStateIDs = new ArrayList<Integer>();
		this.oldDomain = this.trenchGen.generateDomain();
	}
	
	
	public GraphDefinedDomain createGraphDomain() {
		// Get all states and actions.
		List<State> allStates = getAllStatesFromTrenchProb();
		List<Action> allActions = this.oldDomain.getActions();
		GraphDefinedDomain gd = new GraphDefinedDomain(allStates.size());
		setTransitions(allStates, allActions, gd);
		initStateID = allStates.indexOf(this.trenchGen.getInitialState(this.oldDomain));

		return gd;
	}
	
	private void setTransitions(List<State> allStates, List<Action> allActions, GraphDefinedDomain gd) {
		
		TerminalFunction tf = new TrenchDomainGenerator.TrenchTF(this.trenchGen.width - 1, this.trenchGen.height - 1);
		
		
		for (int stateIndex = 0; stateIndex < allStates.size(); stateIndex++) {
			
			if (tf.isTerminal(allStates.get(stateIndex))) {
				// Found the terminal state.
				goalStateIDs.add(stateIndex);
			}
			
			for (int actionIndex = 0; actionIndex < allActions.size(); actionIndex++) {
				
				// Find S, A, S'.
				GroundedAction ga = allActions.get(actionIndex).getAssociatedGroundedAction();
				State nextState = ga.executeIn(allStates.get(stateIndex));
				
				int nextStateNodeID = allStates.indexOf(nextState);
				
				// Set transition.
				gd.setTransition(stateIndex, actionIndex, nextStateNodeID, 1.0);
			}
		}
		
		
	}
	
	private List<State> getAllStatesFromTrenchProb() {
		
		State initialState = this.trenchGen.getInitialState(this.oldDomain);
		RewardFunction rf = new TrenchDomainGenerator.TrenchRF(this.trenchGen.width - 1, this.trenchGen.height - 1);
		TerminalFunction tf = new TrenchDomainGenerator.TrenchTF(this.trenchGen.width - 1, this.trenchGen.height - 1);
		
		double gamma = 0.99;
		int numRollouts = 1;
		double minDelta = 1.0;
		ValueIteration vi = new ValueIteration(this.oldDomain, rf, tf, gamma, new SimpleHashableStateFactory(), minDelta, numRollouts);
		
		GreedyQPolicy policy = vi.planFromState(initialState);
		
		return vi.getAllStates();
	}
	
}
