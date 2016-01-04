package domains;

import graphStateAbstractionTest.VIParams;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
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
		List<State> allNonGraphStates = getAllNonGraphStates();
		List<Action> allActions = this.oldDomain.getActions();

		// Create GraphDefinedDomain.
		GraphDefinedDomain gd = new GraphDefinedDomain(allNonGraphStates.size());

		// Set Transitions.
		setTransitionsAndFindGoalStates(allNonGraphStates, allActions, gd);

		// Set initial state.
		this.initStateID = allNonGraphStates.indexOf(this.oldDomainInitialState);

		return gd;
	}

	private void setTransitionsAndFindGoalStates(List<State> allNonGraphStates, List<Action> allActions, GraphDefinedDomain gd) {
		
		// Loop over each state and set transitions in the graph.
		for (int stateIndex = 0; stateIndex < allNonGraphStates.size(); stateIndex++) {
			this.graphIndexToNonGraphState.put(stateIndex, allNonGraphStates.get(stateIndex));

			// Loop over each action to determine effects of the action.
			for (int actionIndex = 0; actionIndex < allActions.size(); actionIndex++) {
				this.graphActionIndexToNonGraphAction.put(actionIndex, allActions.get(actionIndex));

				// Find S, A, S'.
				GroundedAction ga = allActions.get(actionIndex).getAssociatedGroundedAction();

				if (this.oldDomainTF.isTerminal(allNonGraphStates.get(stateIndex))) {
					// Set terminal transition.
					gd.setTransition(stateIndex, actionIndex, stateIndex, 1.0);
				}
				else {
					List<TransitionProbability> tps = ga.getTransitions(allNonGraphStates.get(stateIndex));
					for (TransitionProbability tp : tps) {
						// Set non-terminal transitions.
						gd.setTransition(stateIndex, actionIndex, allNonGraphStates.indexOf(tp.s), tp.p);
					}
				}
			}
		}
	}

	private List<State> getAllNonGraphStates() {

		// These parameters do *not* matter. Just need to run VI to compute all the reachable states.
		int numRollouts = 1;

		// Run VI.
		ValueIteration vi = new ValueIteration(this.oldDomain, this.oldDomainRF, oldDomainTF, VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, numRollouts);
		vi.toggleReachabiltiyTerminalStatePruning(true);
		GreedyQPolicy policy = vi.planFromState(this.oldDomainInitialState);

		return vi.getAllStates();
	}

}
