package SARealGenerators;

import graphStateAbstraction.VIParams;

import java.util.HashMap;
import java.util.List;

import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.HashableStateFactory;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
/**
 * A type of SARealGenerator which takes in an MDP and produces a map from
 * state, action pairs to Q values by running value iteration.
 * @author Dhershkowitz
 *
 */
public class qValueGenerator extends SARealGenerator {

	HashMap<SAPair, Double> realsMap;
	public qValueGenerator(Domain d, RewardFunction rf, State initialState) {
		super(d, rf, initialState);
		
		this.realsMap = this.getRealsMap();
	}


	/**
	 * 
	 * @param d
	 * @param rf
	 * @param initialState
	 */


	@Override
	public HashMap<SAPair, Double> getRealsMap() {
		if (realsMap != null) return realsMap;
		else {
			HashMap<SAPair, Double> toReturn = new HashMap<SAPair, Double>();
			TerminalFunction tf = new NullTermination();
			HashableStateFactory hf = new SimpleHashableStateFactory();
			ValueIteration VI = new ValueIteration(d, rf, tf, VIParams.gamma, hf, VIParams.maxDelta, VIParams.maxIterations);
			VI.planFromState(initialState);
			for (State s : VI.getAllStates()) {
				for (Action a : VI.getActions()) {
					SAPair sa = new SAPair(s, a);
					toReturn.put(sa, VI.getQ(s, a.getAssociatedGroundedAction()).q);
				}
			}
			return toReturn;
		}
	}


}
