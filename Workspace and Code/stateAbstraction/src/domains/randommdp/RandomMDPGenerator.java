package domains.randommdp;

import java.util.Random;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class RandomMDPGenerator {
	/**
	 * 
	 * @param totalNumStates
	 * @return an n state graph MDP.
	 */
	public static GraphDefinedDomain getRandomMDP(int numStates,  int numActions) {
		GraphDefinedDomain dg = new GraphDefinedDomain(numStates);
		Random rand = new Random();

		// Each action randomly transitions to two states for each state.
		for (int actionIndex = 0; actionIndex < numActions; actionIndex++) {
			for (int currentStateIndex = 0; currentStateIndex < numStates; currentStateIndex++) {
				int firstRandomStateIndex = rand.nextInt(numStates);
				int secondRandomStateIndex = rand.nextInt(numStates);

				((GraphDefinedDomain) dg).setTransition(currentStateIndex, actionIndex/*action*/, firstRandomStateIndex, .5);
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, actionIndex/*action*/, secondRandomStateIndex, .5);
			}
		}

		return dg;
	}	
	
	public static State getInitialState(Domain d) {
		return GraphDefinedDomain.getState(d, 0);
	}

	/**
	 * n State chain reward function -- returns 10 if action 1 is taken in the last state, 2 if
	 * action 0 is taken ever and 0 otherwise.
	 * @author Dhershkowitz
	 *
	 */
	public static class RandomMDPRF implements RewardFunction {

		double[][] rewardMatrix;


		public RandomMDPRF(int numStates) {

			//Set up random reward matrix
			rewardMatrix = new double[numStates][numStates];
			for (int i = 0; i < numStates; i++) {
				for (int j = 0; j < numStates; j++) {
					if (Math.random() > .5) {// 50% prob reward
						rewardMatrix[i][j] = Math.random(); // 0-1 reward uniform
					}

				}
			}
		}

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			int sIndex = GraphDefinedDomain.getNodeId(s);
			int sprimeIndex = GraphDefinedDomain.getNodeId(sprime);
			return rewardMatrix[sIndex][sprimeIndex];
		}
	}


}
