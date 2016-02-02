package domains.minefield;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class MinefieldGenerator {
	
	private static double slipProbability = 0.01;
	
	/**
	 * 
	 * @param width
	 * @param height
	 * @return
	 */
	public static GraphDefinedDomain getMinefield(int width, int height) {
		GraphDefinedDomain dg = new GraphDefinedDomain(height*width);
		
		// Set up transitions for left/right.
		for (int j = 0; j < height; j++){
			for (int i = 0; i < width; i++) {
				int currentStateIndex = i +j*width;

				int leftIndex = 0;
				if (i==0) leftIndex =  currentStateIndex;
				else leftIndex = currentStateIndex-1;

				int rightIndex = 0;
				if (i == width-1) rightIndex = currentStateIndex;
				else rightIndex = currentStateIndex+1;

				//Set up left action.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*action*/, leftIndex, 1.0 - slipProbability);
				
				//Set up right action.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 1/*action*/, rightIndex, 1.0 - slipProbability);
				
				// Set slip probs for Up:
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 2/*up action*/, leftIndex, slipProbability/2.0);
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 2/*up action*/, rightIndex, slipProbability/2.0);
				
				// Set slip probs for down:
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 3/*down action*/, leftIndex, slipProbability/2.0);
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 3/*down action*/, rightIndex, slipProbability/2.0);
			}
		}

		// Set up transition for up.
		for (int j = 0; j < height-1; j++){
			for (int i = 0; i < width; i++) {
				int currentStateIndex = i +j*width;
				// Set normal transition for up.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 2/*up action*/, currentStateIndex+width, 1.0 - slipProbability);
				
				// Set slip for Left and Right
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*left action*/, currentStateIndex+width, slipProbability/2.0);
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 1/*right action*/, currentStateIndex+width, slipProbability/2.0);
			}
		}
		// Set up transition for down.
		for (int j = 0; j < height-1; j++){
			for (int i = 0; i < width; i++) {
				int currentStateIndex = i +j*width;
				int stateBelowIndex = currentStateIndex-width;
				if (stateBelowIndex < 0) stateBelowIndex = currentStateIndex; // Self loop at bottom
				System.out.println("sbelow: " + stateBelowIndex);
				
				// Set normal transition for down.
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 3/*down action*/, stateBelowIndex, 1.0 - slipProbability);
				
				// Set slip for Left and Right
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*left action*/, stateBelowIndex, slipProbability/2.0);
				((GraphDefinedDomain) dg).setTransition(currentStateIndex, 1/*right action*/, stateBelowIndex, slipProbability/2.0);
			}
		}

		// Set up self-loops at the end.
		for (int i = 0; i < width; i++) {
			int j = height-1;
			int currentStateIndex = i +j*width;
			((GraphDefinedDomain) dg).setTransition(currentStateIndex, 2/*action*/, currentStateIndex, 1.0);
		}

		return dg;
	}	

	/**
	 * Minefield reward function -- returns 10 if action 1 is taken in the last state, 2 if
	 * action 0 is taken ever and 0 otherwise.
	 * @author Dhershkowitz
	 *
	 */
	public static class MinefieldRF implements RewardFunction {

		private List<Integer> mineStates = new ArrayList<Integer>();
		private Random prf = new Random();
		
		public MinefieldRF(int numMineStates, int numTotalStates) {
			for (int i = 0; i < numMineStates; i++) {
				mineStates.add(prf.nextInt(numTotalStates));
			}
		}
		
		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			if (s.equals(sprime) && a.actionName().equals("action2")){ 
				return 1;
			}
			else if (mineStates.contains(sprime)) {
				return 0;
			} else{
				return 0.1;
			}
		}

	}
}
