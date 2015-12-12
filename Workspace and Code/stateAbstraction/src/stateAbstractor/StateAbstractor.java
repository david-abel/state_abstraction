package stateAbstractor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import cern.colt.Arrays;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.AbstractGroundedAction;
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
	private HashMap<Integer, Integer> groundToAbstractState;

	/**
	 * 
	 * @param inpDG the ground MDP domain generator
	 * @param rf the ground MDP reward function
	 */
	public GraphDefinedDomain abstractMDP(GraphDefinedDomain inpDG, RewardFunction rf) {
		Domain inputD = inpDG.generateDomain();
		HashableStateFactory hf = new SimpleHashableStateFactory();

		HashMap<State, List<State>> stateToPhiedStates = new HashMap<State, List<State>>();
		// Get all ground states in random shuffled order.
		List<Integer> indexList = new ArrayList<Integer>();
		for (int stateIndex = 0; stateIndex < inpDG.getNumNodes(); stateIndex++) {
			indexList.add(stateIndex);
		}
		Collections.shuffle(indexList);
		
		for (int stateIndex : indexList) {
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
					//Loop over ground states.
					for (State groundState : abstractStateIndexToGroundStates.get(abstractStateIndex)) {
						double numStatesInCluster = abstractStateIndexToGroundStates.get(abstractStateIndex).size();
						for (State otherGroundedState : abstractStateIndexToGroundStates.get(otherAbstractStateIndex)) {
							double weightOfGroundState = 1.0/numStatesInCluster; // TODO: Don't just assume uniform weighting

							//Add to transition prob based on weighting scheme.
							double transitionProbInGroundState = 0;
							List<TransitionProbability> gsgaTransitions = currGA.getTransitions(groundState);
							boolean transitionBetweenGroundStates = false;
							for (TransitionProbability tp : gsgaTransitions) {
								if (tp.s.equals(otherGroundedState)) {
									transitionProbInGroundState = tp.p;
									transitionBetweenGroundStates = true;
								}
							}
							if (!transitionBetweenGroundStates) continue;

							transitionProbability += weightOfGroundState*transitionProbInGroundState;

							//Add to rewards
							double groundReward = rf.reward(groundState, currGA, otherGroundedState);
							rewardMatrix.get(currGA)[abstractStateIndex][otherAbstractStateIndex] += weightOfGroundState*groundReward;
						}
					}
						if (transitionProbability != 0) toReturn.setTransition(abstractStateIndex, actionIndex, otherAbstractStateIndex, transitionProbability);
				}
			}
		}
		this.rf = new abstractRewardMatrix(rewardMatrix);

		// Set up mapping from ground MDP states to abstract MDP states (used later to get ground policy).
		this.groundToAbstractState = new HashMap<Integer, Integer>();
		for (Integer abstractStateIndex : abstractStateIndexToGroundStates.keySet()) {
			List<State> groundStates = abstractStateIndexToGroundStates.get(abstractStateIndex);
			for (State sg : groundStates) {
				Integer sgInt = GraphDefinedDomain.getNodeId(sg);
				this.groundToAbstractState.put(sgInt, abstractStateIndex);
			}
		}
		
		return toReturn;
	}
	
	public State getAbstractInitialState(Domain abstractDomain, State groundInitialState) {
		int groundIndex = GraphDefinedDomain.getNodeId(groundInitialState);
		int abstractInitialStateIndex = this.groundToAbstractState.get(groundIndex);
		return GraphDefinedDomain.getState(abstractDomain, abstractInitialStateIndex);
	}
	
	/**
	 * Uses VI to solve for optimal policy in abstract state, then
	 * returns the policy for ground MDP as the abstract policy dictates.
	 * @return policy for ground MDP
	 */
	public Policy getPolicyForGroundMDP(GreedyQPolicy abstractPolicy, Domain abstractDomain, Domain groundDomain) {
		return new PolicyForGroundMDP(this.groundToAbstractState, abstractPolicy, groundDomain, abstractDomain);
	}
	
	private class PolicyForGroundMDP extends Policy {
		
		private HashMap<Integer, Integer> groundToAbstract; // Using integers b/c states don't hash w/o factory
		private GreedyQPolicy abstractPolicy;
		private Domain aD;
		private Domain gD;
		
		public PolicyForGroundMDP(HashMap<Integer, Integer> groundToAbstract, GreedyQPolicy abstractPolicy, Domain gD, Domain aD) {
			this.groundToAbstract= groundToAbstract;
			this.abstractPolicy = abstractPolicy;
			this.aD = aD;
			this.gD = gD;
		}

		@Override
		public AbstractGroundedAction getAction(State s) {
			Integer groundStateInteger = GraphDefinedDomain.getNodeId(s);
			Integer abstractStateInteger = this.groundToAbstract.get(groundStateInteger);
			State abstractState = GraphDefinedDomain.getState(this.aD, abstractStateInteger);
			AbstractGroundedAction toReturn = this.abstractPolicy.getAction(abstractState);
			return toReturn;
		}

		@Override
		public List<ActionProb> getActionDistributionForState(State s) {
			return this.getDeterministicPolicy(s);
		}

		@Override
		public boolean isStochastic() {
			return false;
		}

		@Override
		public boolean isDefinedFor(State s) {
			return true;
		}
		
	}

	/**
	 * 
	 * @return the abstract reward function that results from collapsing the input MDP
	 */
	public RewardFunction getRewardFunction() {
		return this.rf;
	}

	/**
	 * An inner class to help construct the abstract reward function.
	 * @author Dhershkowitz
	 *
	 */
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


	/**
	 * Returns whether or not this state abstractor considers the input states
	 * equivalent and therefore in need of collapse.
	 * Must be an equivalence relation. Currently must be deterministic.
	 */
	protected abstract boolean phi(State s1, State s2);


}


