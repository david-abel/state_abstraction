package stateAbstractor;

import graphStateAbstractionTest.VIParams;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import cern.colt.Arrays;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.EpisodeAnalysis;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.core.AbstractGroundedAction;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.TransitionProbability;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.HashableState;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;

/**
 * Class used to convert ground MDPs into abstract MDPs.
 * @author Dhershkowitz
 *
 */
public abstract class StateAbstractor {

	private RewardFunction aRF;
	private HashMap<Integer, Integer> groundToAbstractState;
	private HashMap<Integer, List<Integer>> abstractIndexToGroundIndex;

	/**
	 * 
	 * @param inpDG
	 * @param groundRF
	 * @param groundTF
	 * @param initialGroundState
	 * @return a GraphDefinedDomain of the resulting abstractMDP
	 */
	public GraphDefinedDomain abstractMDP(GraphDefinedDomain inpDG, RewardFunction groundRF, TerminalFunction groundTF, State initialGroundState) {
		Domain groundD = inpDG.generateDomain();

		//Use the input phi and epsilon to group together ground states greedily. Returns mapping from abstract state index to list of ground states.
		HashMap<Integer, List<State>> abstractStateIndexToGroundStates = setupGroupingsOfGroundStatesIntoAbstractStates(inpDG, groundD);

		//Setup map from abstract state index to ground indices for public function.
		setupAbstractIndexToGroundIndexMap(abstractStateIndexToGroundStates);

		// Set up mapping from ground MDP states to abstract MDP states (used later to get ground policy).
		setupGroundIndexToAbstractStateIndexMap(abstractStateIndexToGroundStates);

		return setupAbstractDomainAndAbstractRewardFunctionAndAbstractTerminalFunction(groundD, initialGroundState, groundRF, groundTF, abstractStateIndexToGroundStates);
	}

	/**
	 * 
	 * @param groundDomain the input ground MDP domain
	 * @param groundStateIndexList 
	 * @return
	 */
	private HashMap<Integer, List<State>> setupGroupingsOfGroundStatesIntoAbstractStates(GraphDefinedDomain groundDG, Domain groundDomain) {
		// Get all ground states in random shuffled order.
		List<Integer> groundStateIndexList = new ArrayList<Integer>();
		for (int stateIndex = 0; stateIndex < groundDG.getNumNodes(); stateIndex++) {
			groundStateIndexList.add(stateIndex);
		}
		Collections.shuffle(groundStateIndexList);


		HashMap<State, List<State>> stateToPhiedStates = new HashMap<State, List<State>>();

		for (int stateIndex : groundStateIndexList) {
			State currGroundState = GraphDefinedDomain.getState(groundDomain, stateIndex);
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

		return abstractStateIndexToGroundStates;
	}

	private void setupAbstractIndexToGroundIndexMap(HashMap<Integer, List<State>> abstractStateIndexToGroundStates) {
		this.abstractIndexToGroundIndex = new HashMap<Integer, List<Integer>>();
		for (int abstractStateIndex = 0; abstractStateIndex < abstractStateIndexToGroundStates.size(); abstractStateIndex++) {
			List<Integer> groundStateIndices = new ArrayList<Integer>();

			for (State groundState : abstractStateIndexToGroundStates.get(abstractStateIndex)) {
				Integer groundStateIndex = GraphDefinedDomain.getNodeId(groundState);
				groundStateIndices.add(groundStateIndex);
			}
			Collections.sort(groundStateIndices);
			this.abstractIndexToGroundIndex.put(abstractStateIndex, groundStateIndices);
		}

	}

	private void setupGroundIndexToAbstractStateIndexMap(HashMap<Integer, List<State>> abstractStateIndexToGroundStates) {
		this.groundToAbstractState = new HashMap<Integer, Integer>();
		for (Integer abstractStateIndex : abstractStateIndexToGroundStates.keySet()) {
			List<State> groundStates = abstractStateIndexToGroundStates.get(abstractStateIndex);
			for (State sg : groundStates) {
				Integer sgInt = GraphDefinedDomain.getNodeId(sg);
				this.groundToAbstractState.put(sgInt, abstractStateIndex);
			}
		}
	}

	private GraphDefinedDomain setupAbstractDomainAndAbstractRewardFunctionAndAbstractTerminalFunction(Domain groundD, State initialState, RewardFunction groundRF, TerminalFunction groundTF, HashMap<Integer, List<State>> abstractStateIndexToGroundStates) {
		HashableStateFactory hf = new SimpleHashableStateFactory();

		//Instantiate abstract MDP with num states as clusters.
		GraphDefinedDomain toReturn = new GraphDefinedDomain(abstractStateIndexToGroundStates.keySet().size());

		// Initialize transition and reward matrix for abstract MDP.
		HashMap<GroundedAction, double[][]> rewardMatrix = new HashMap<GroundedAction, double[][]>();
		HashMap<Integer, double[][]> transitionMatrix = new HashMap<Integer, double[][]>();//Maps from action index to transition matrix
		int i = 0;
		for (Action a : groundD.getActions()) {
			GroundedAction ga = a.getAssociatedGroundedAction();
			rewardMatrix.put(ga, new double[toReturn.getNumNodes()][toReturn.getNumNodes()]);
			transitionMatrix.put(i, new double[toReturn.getNumNodes()][toReturn.getNumNodes()]);
			i++;
		}

		//Loop over abstract states.
		for (int abstractStateIndex = 0; abstractStateIndex < toReturn.getNumNodes(); abstractStateIndex++) {
			double numStatesInCluster = abstractStateIndexToGroundStates.get(abstractStateIndex).size();

				//Loop over ground states in the current abstract state.
				for (State groundState : abstractStateIndexToGroundStates.get(abstractStateIndex)) {
					double weightOfGroundState = 1.0/numStatesInCluster; //getSteadyStateWeighting(groundD, groundState, abstractStateIndexToGroundStates.get(abstractStateIndex), hf, initialState, groundRF, groundTF);

					//Loop over actions.
					for (int actionIndex = 0; actionIndex < groundD.getActions().size(); actionIndex++) {
						//Add to transition matrix.
						GroundedAction currGA = groundD.getActions().get(actionIndex).getAssociatedGroundedAction();
						List<TransitionProbability> gsgaTransitions = currGA.getTransitions(groundState);
						for (TransitionProbability tp : gsgaTransitions) {
							double toAdd = weightOfGroundState*tp.p;
							int abstractStateIndexOfArrivedInState = this.groundToAbstractState.get(GraphDefinedDomain.getNodeId(tp.s));
							transitionMatrix.get(actionIndex)[abstractStateIndex][abstractStateIndexOfArrivedInState] += toAdd;
						}


						//Loop over other abstract states.
						for (int otherAbstractStateIndex = 0; otherAbstractStateIndex < toReturn.getNumNodes(); otherAbstractStateIndex++) {

						//Loop over ground states for the other abstract state.
						for (State otherGroundedState : abstractStateIndexToGroundStates.get(otherAbstractStateIndex)) {

							//Add to rewards.
							double groundReward = groundRF.reward(groundState, currGA, otherGroundedState);
							rewardMatrix.get(currGA)[abstractStateIndex][otherAbstractStateIndex] += weightOfGroundState*groundReward;
						}

					}
				}
			}
		}
		//Use transition matrix to create transition dynamics.
		for (Entry<Integer, double[][]> kv : transitionMatrix.entrySet()) {
			int actionIndex = kv.getKey();
			double [][] transitionMatrixForAction = kv.getValue();
			for (int fromAbstractStateIndex = 0; fromAbstractStateIndex < transitionMatrixForAction.length; fromAbstractStateIndex++) {
				for (int toAbstractStateIndex = 0; toAbstractStateIndex < transitionMatrixForAction.length; toAbstractStateIndex++) {
					double probabilityOfTransition = transitionMatrixForAction[fromAbstractStateIndex][toAbstractStateIndex];
					if (probabilityOfTransition != 0) {
						toReturn.setTransition(fromAbstractStateIndex, actionIndex, toAbstractStateIndex, probabilityOfTransition);
					}
				}
			}
		}

		//Use reward matrix to create reward function.
		this.aRF = new abstractRewardMatrix(rewardMatrix);

		return toReturn;
	}

	private HashMap<HashableState, Double> steadyStateMapping = null;
	/**
	 * A helper for setting up the TF and RF. Used to weight ground states which contribute to an abstract state.
	 * @param d
	 * @param s
	 * @param allStatesInCluster
	 * @param hf
	 * @param initialState
	 * @param rf
	 * @param tf
	 * @return
	 */
	private double getSteadyStateWeighting(Domain d, State s, List<State> allStatesInCluster, HashableStateFactory hf, State initialState, RewardFunction groundRF, TerminalFunction groundTF) {
		HashableState hs = hf.hashState(s);
		//If steadyStateMapping not yet set up, do it.
		if (steadyStateMapping == null) {
			setUpSteadyStateMapping(d, initialState, groundTF, groundRF, hf);
		}

		//Sum up steady state dist across all states in cluster and then return percentage the input state contributes 
		double totalProbMassOfGroundStatesInCluster = 0.0;
		for (State stateInCluster : allStatesInCluster) {
			HashableState hsO = hf.hashState(stateInCluster);
			totalProbMassOfGroundStatesInCluster += steadyStateMapping.getOrDefault(hsO, new Double(0));
		}
		double toReturn = steadyStateMapping.getOrDefault(hs, new Double(0))/totalProbMassOfGroundStatesInCluster;
		if (totalProbMassOfGroundStatesInCluster == 0.0) return 1.0/allStatesInCluster.size(); //Return uniform weighting if probability is 0 for all
		System.out.println(toReturn);

		return toReturn;
	}

	private void setUpSteadyStateMapping(Domain d, State initialState, TerminalFunction tf, RewardFunction rf, HashableStateFactory hf) {
		this.steadyStateMapping = new HashMap<HashableState, Double>();
		ValueIteration VI = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
		GreedyQPolicy pol = VI.planFromState(initialState);
		//Perform rollouts
		int numRollouts = 100;
		int maxNumSteps = 1000;
		double totalNumberOfStatesVisited = 0;
		for (int i = 0; i < numRollouts; i++) {
			EpisodeAnalysis ea = pol.evaluateBehavior(initialState, rf, maxNumSteps);
			//Increment states visited
			for (State s : ea.stateSequence) {
				HashableState hs = hf.hashState(s);
				Double oldValue = steadyStateMapping.getOrDefault(hs, new Double(0));
				steadyStateMapping.put(hs, (oldValue+1));
				totalNumberOfStatesVisited += 1;
			}
		}
		//Normalize by sum
		for (Entry<HashableState, Double> kvPair : steadyStateMapping.entrySet()) {
			double normValue = kvPair.getValue()/totalNumberOfStatesVisited;
			if (totalNumberOfStatesVisited == 0.0) {
				normValue = 0.0;
			}
			steadyStateMapping.put(kvPair.getKey(), normValue);
		}

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
		return this.aRF;
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
		@Override
		public String toString() {
			StringBuilder sb = new StringBuilder();
			for (Entry<GroundedAction, double[][]> kvPair : rewardMatrix.entrySet()) {
				sb.append(kvPair.getKey().actionName() + ":\n");
				for (double[] rewards: kvPair.getValue()) {
					sb.append(Arrays.toString(rewards) + "\n");
				}
			}

			return sb.toString();
		}
	}

	public List<Integer> getGroundIndicesFromAbstractIndex(int abstractIndex) {
		return this.abstractIndexToGroundIndex.get(abstractIndex);
	}


	/**
	 * Returns whether or not this state abstractor considers the input states
	 * equivalent and therefore in need of collapse.
	 * Must be an equivalence relation. Currently must be deterministic.
	 */
	protected abstract boolean phi(State s1, State s2);


}


