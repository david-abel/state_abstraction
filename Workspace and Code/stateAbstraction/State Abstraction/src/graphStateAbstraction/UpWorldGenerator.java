package graphStateAbstraction;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class UpWorldGenerator {
	/**
	 * 
	 * @param totalNumStates
	 * @return an n state graph MDP.
	 */
	public static GraphDefinedDomain getUPWorld(int width, int height) {
		GraphDefinedDomain dg = new GraphDefinedDomain(height*width);

		// Set up transitions for left/right.
		for (int j = 0; j < height; j++){
			for (int i = 0; i < width; i++) {
				int currentStateIndex = i +j*width;

				//Set up left action.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*action*/, ((currentStateIndex-1)+width)%width, 1.0);

				//Set up right action.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 1/*action*/, (currentStateIndex+1)%width, 1.0);
			}
		}

		// Set up transition for up.
		for (int j = 0; j < height-1; j++){
			for (int i = 0; i < width; i++) {
				int currentStateIndex = i +j*width;
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*action*/, currentStateIndex+width, 1.0);
			}
		}

		// Set up self-loops at the end.
		for (int i = 0; i < width; i++) {
			int j = height-1;
			int currentStateIndex = i +j*width;
			((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*action*/, currentStateIndex, 1.0);
		}

		return dg;
	}	

	/**
	 * n State chain reward function -- returns 10 if action 1 is taken in the last state, 2 if
	 * action 0 is taken ever and 0 otherwise.
	 * @author Dhershkowitz
	 *
	 */
	public static class UpWorldRF implements RewardFunction {

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			if (s.equals(sprime)) return 10;
			else return 0;
		}

		}
	}
