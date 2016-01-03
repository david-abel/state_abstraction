package domains;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public class NormalDomainToGraphDomain {

	DomainGenerator domainGen;
	TerminalFunction oldDomainTF;
	RewardFunction oldDomainRF;
	State oldDomainInitialState;
	Domain oldDomain;
	public int initStateID;
	public HashMap<Integer, State> graphIndexToNonGraphState;
	public HashMap<Integer, Action> graphActionIndexToNonGraphAction;
	
	public NormalDomainToGraphDomain(DomainGenerator tg, TerminalFunction originalTF, RewardFunction originalRF, State initState) {
		this.domainGen = tg;
		this.oldDomainTF = originalTF;
		this.oldDomainRF = originalRF;
		this.oldDomainInitialState = initState;
		this.oldDomain = this.domainGen.generateDomain();
		this.graphIndexToNonGraphState = new HashMap<Integer, State>();
		this.graphActionIndexToNonGraphAction = new HashMap<Integer, Action>();
	}
	
	
	public GraphDefinedDomain createGraphDomain() {
		// Get all states and actions.
		List<State> allStates = getAllStates();
		List<Action> allActions = this.oldDomain.getActions();
		
		// Create GraphDefinedDomain.
		GraphDefinedDomain gd = new GraphDefinedDomain(allStates.size());
		
		// Set Transitions.
		setTransitionsAndFindGoalStates(allStates, allActions, gd);
		
		// Set initial state.
		initStateID = allStates.indexOf(this.oldDomainInitialState);

		return gd;
	}
	
	private void setTransitionsAndFindGoalStates(List<State> allStates, List<Action> allActions, GraphDefinedDomain gd) {
		
		// Loop over each state and set transitions in the graph.
		for (int stateIndex = 0; stateIndex < allStates.size(); stateIndex++) {
			this.graphIndexToNonGraphState.put(stateIndex, allStates.get(stateIndex));

			// Loop over each action to determine effects of the action.
			for (int actionIndex = 0; actionIndex < allActions.size(); actionIndex++) {
				this.graphActionIndexToNonGraphAction.put(actionIndex, allActions.get(actionIndex));
				
				// Find S, A, S'.
				GroundedAction ga = allActions.get(actionIndex).getAssociatedGroundedAction();
				State nextState = ga.executeIn(allStates.get(stateIndex));
				int nextStateNodeID = allStates.indexOf(nextState);
				

				if (this.oldDomainTF.isTerminal(allStates.get(stateIndex))) {
					// Set terminal transition.
					gd.setTransition(stateIndex, actionIndex, stateIndex, 1.0);
				}
				else {
					// Set non-terminal transition.
					gd.setTransition(stateIndex, actionIndex, nextStateNodeID, 1.0);
					
				}
				
			}
			
			
		}
	}
	
	private List<State> getAllStates() {
		
		// Set initial state and reward function.
		State initialState = this.oldDomainInitialState;
		RewardFunction rf = this.oldDomainRF;
		
		// These parameters do *not* matter. Just need to run VI to compute all the reachable states.
		double gamma = 0.99;
		int numRollouts = 1;
		double minDelta = 1.0;
		
		// Run VI.
		ValueIteration vi = new ValueIteration(this.oldDomain, rf, oldDomainTF, gamma, new SimpleHashableStateFactory(), minDelta, numRollouts);
		GreedyQPolicy policy = vi.planFromState(initialState);
		
		return vi.getAllStates();
	}
	
}
