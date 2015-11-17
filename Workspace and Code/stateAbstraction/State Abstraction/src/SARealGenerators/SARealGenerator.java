package SARealGenerators;

import java.util.HashMap;
import java.util.List;

import burlap.oomdp.core.Domain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.RewardFunction;

/**
 * A class used to generate real values on state, action pairs to form
 * the basis of some abstraction. Is fed to PhiSAReal to help it to determine
 * if it should collapse two states.
 * @author Dhershkowitz
 *
 */
public abstract class SARealGenerator {
	protected Domain d;
	protected RewardFunction rf;
	protected double gamma;
	protected State initialState;
	
	/**
	 * 
	 * @param d
	 * @param rf
	 * @param gamma
	 * @param initialState
	 */
	public SARealGenerator(Domain d, RewardFunction rf, double gamma, State initialState) {
		this.d = d;
		this.rf = rf;
		this.gamma = gamma;
		this.initialState = initialState;
	}
	
	/**
	 * 
	 * @return The mapping from SAPairs to the real value function associated with
	 * this SARealGenerator. E.g. qValueGenerator produces a map from state, action
	 * pairs to Q values.
	 */
	public abstract HashMap<SAPair, Double> getRealsMap();
}
