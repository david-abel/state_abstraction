package graphStateAbstraction;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.HashableState;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

public abstract class StateAbstractor {

	private RewardFunction rf;

	public GraphDefinedDomain abstractMDP(GraphDefinedDomain inpDG, RewardFunction rf) {
		Domain inputD = inpDG.generateDomain();
		HashableStateFactory hf = new SimpleHashableStateFactory();

		HashMap<State, List<State>> stateToPhiedStates = new HashMap<State, List<State>>();
		// Get all ground states.
		for (int stateIndex = 0; stateIndex < inpDG.getNumNodes(); stateIndex++) {
			State currGroundState = GraphDefinedDomain.getState(inputD, stateIndex);
			boolean foundACluster = false;
			for (State currGroundStateRep : stateToPhiedStates.keySet()) {
				// Should add to this cluster.
				if (this.phi(currGroundState, currGroundStateRep)) {
					List<State> stateCluster = stateToPhiedStates.get(currGroundStateRep);
					stateCluster.add(currGroundState);
					foundACluster = true;
				}
			}
			// No existing cluster for this state.
			if (!foundACluster) {
				List<State> newCluster = new ArrayList<State>();
				newCluster.add(currGroundState);
				stateToPhiedStates.put(currGroundState, newCluster);
			}
		}

		//Get map from abstract state index to clustered ground states.
		HashMap<Integer, List<State>> abstractStateIndexToGroundStates = new HashMap<Integer, List<State>>();
		List<List<State>> clusters = new ArrayList<List<State>>();
		for (List<State> cluster : stateToPhiedStates.values()) {
			clusters.add(cluster);
		}

		for (int abstractStateIndex = 0; abstractStateIndex < clusters.size(); abstractStateIndex++) {
			abstractStateIndexToGroundStates.put(abstractStateIndex, clusters.get(abstractStateIndex));
		}


		//Instantiate abstract MDP with num states as clusters.
		GraphDefinedDomain toReturn = new GraphDefinedDomain(stateToPhiedStates.keySet().size());

		// Initialize reward matrix.
		HashMap<GroundedAction, double[][]> rewardMatrix = new HashMap<GroundedAction, double[][]>();
		for (Action a : inputD.getActions()) {
			GroundedAction ga = a.getAssociatedGroundedAction();
			rewardMatrix.put(ga, new double[toReturn.getNumNodes()][toReturn.getNumNodes()]);
		}

		//Set up new transitions and rewards in abstracted MDP.
		for (int abstractStateIndex = 0; abstractStateIndex < toReturn.getNumNodes(); abstractStateIndex++) {
			for (int otherAbstractStateIndex = 0; otherAbstractStateIndex < toReturn.getNumNodes(); otherAbstractStateIndex++) {
				for (int actionIndex = 0; actionIndex < inputD.getActions().size(); actionIndex++) {
					GroundedAction currGA = inputD.getActions().get(actionIndex).getAssociatedGroundedAction();
					double transitionProbability = 0.0;
					for (State groundState : abstractStateIndexToGroundStates.get(abstractStateIndex)) {
						double numStatesInCluster = abstractStateIndexToGroundStates.get(abstractStateIndex).size();
						for (State otherGroundedState : abstractStateIndexToGroundStates.get(otherAbstractStateIndex)) {
							//							HashableState hashedOGS = hf.hashState(otherGroundedState);
							double weightOfGroundState = 1.0/numStatesInCluster; // ASSUMING UNIFORM WEIGHTING

							//Add to transition prob based on weighting scheme.
							double transitionProbInGroundState = 0;
							List<TransitionProbability> gsgaTransitions = currGA.getTransitions(groundState);
							for (TransitionProbability tp : gsgaTransitions) {
								if (tp.s.equals(otherGroundedState)) {
									transitionProbInGroundState = tp.p;
								}
							}
							transitionProbability += weightOfGroundState*transitionProbInGroundState;

							//Add to rewards
							double oldReward = rf.reward(groundState, currGA, otherGroundedState);
							rewardMatrix.get(currGA)[abstractStateIndex][otherAbstractStateIndex] += weightOfGroundState*oldReward;
						}
						toReturn.setTransition(abstractStateIndex, actionIndex, otherAbstractStateIndex, transitionProbability);
					}
				}
			}
		}
		this.rf = new abstractRewardMatrix(rewardMatrix);
		return toReturn;
	}

	public RewardFunction getRewardFunction() {
		return this.rf;
	}

	private static class abstractRewardMatrix implements RewardFunction {
		HashMap<GroundedAction, double[][]> rewardMatrix;

		public abstractRewardMatrix(HashMap<GroundedAction, double[][]> rewardMatrix) {
			this.rewardMatrix = rewardMatrix;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			int sIndex = GraphDefinedDomain.getNodeId(s);
			int sPrimeIndex = GraphDefinedDomain.getNodeId(sprime);
			double toReturn = rewardMatrix.get(a)[sIndex][sPrimeIndex];
			return toReturn;
		}


	}


	/*
	 * Must be an equivalence relation. Currently must be deterministic.
	 */
	protected abstract boolean phi(State s1, State s2);


}


