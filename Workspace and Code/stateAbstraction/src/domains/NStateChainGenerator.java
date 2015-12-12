package domains;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

/**
 * A class with static methods to instantiate nstate problems and nstate chain reward functions.
 * @author Dhershkowitz
 *
 */
public class NStateChainGenerator {
	/**
	 * 
	 * @param totalNumStates
	 * @return an n state graph MDP.
	 */
	public static GraphDefinedDomain getNStateChain(int totalNumStates) {
		double slipProb = .2;

		GraphDefinedDomain dg = new GraphDefinedDomain(totalNumStates);

		// Set up transitions
		for (int i = 0; i < totalNumStates-1; i++) {
			((GraphDefinedDomain) dg).setTransition(i, 0/*action*/, i+1, slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 0/*action*/, 0, 1.0-slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 1/*action*/, 0, slipProb);
			((GraphDefinedDomain) dg).setTransition(i, 1/*action*/, i+1, 1.0-slipProb);
		}

		// Set up last state transition
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 0/*action*/, totalNumStates-1, slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 0/*action*/, 0,1.0-slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, totalNumStates-1, 1.0-slipProb);
		((GraphDefinedDomain) dg).setTransition(totalNumStates-1, 1/*action*/, 0, slipProb);

		return dg;
	}	
	
	/**
	 * n State chain reward function -- returns 10 if action 1 is taken in the last state, 2 if
	 * action 0 is taken ever and 0 otherwise.
	 * @author Dhershkowitz
	 *
	 */
	public static class nStateChainRF implements RewardFunction {
		int n;
		
		public nStateChainRF(int n) {
			this.n = n;
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			if (a.actionName().equals("action0")) {
				return 2;
			}
			else {
				if (GraphDefinedDomain.getNodeId(s) == n-1 && GraphDefinedDomain.getNodeId(sprime) == n-1) {
					return 10;
				}
			}
			return 0;
		}
		
	}
}
