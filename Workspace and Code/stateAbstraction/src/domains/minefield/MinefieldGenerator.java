package domains.minefield;

import java.util.ArrayList;
import java.util.HashMap;
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

				int aboveIndex = currentStateIndex+width;
				if (aboveIndex >= height*width) aboveIndex = currentStateIndex;

				int belowIndex = currentStateIndex-width;
				if (belowIndex < 0) belowIndex = currentStateIndex; // Self loop at bottom



				//Left
				HashMap<Integer, Double> leftHM = new HashMap<Integer, Double>();
				addToExistingValue(leftHM, leftIndex, 1.0-slipProbability);
				addToExistingValue(leftHM, aboveIndex, slipProbability/2.0);
				addToExistingValue(leftHM, belowIndex, slipProbability/2.0);
				//Convert HM to transitions
				for (Integer resultingStateIndex : leftHM.keySet()) {
					((GraphDefinedDomain) dg).setTransition(currentStateIndex, 0/*left action*/, resultingStateIndex, leftHM.get(resultingStateIndex));
				}

				//Right
				HashMap<Integer, Double> rightHM = new HashMap<Integer, Double>();
				addToExistingValue(rightHM, rightIndex, 1.0-slipProbability);
				addToExistingValue(rightHM, aboveIndex, slipProbability/2.0);
				addToExistingValue(rightHM, belowIndex, slipProbability/2.0);
				//Convert HM to transitions
				for (Integer resultingStateIndex : rightHM.keySet()) {
					((GraphDefinedDomain) dg).setTransition(currentStateIndex, 1/*right action*/, resultingStateIndex, rightHM.get(resultingStateIndex));
				}

				//Down
				HashMap<Integer, Double> downHM = new HashMap<Integer, Double>();
				addToExistingValue(downHM, belowIndex, 1.0-slipProbability);
				addToExistingValue(downHM, rightIndex, slipProbability/2.0);
				addToExistingValue(downHM, leftIndex, slipProbability/2.0);
				for (Integer resultingStateIndex : downHM.keySet()) {
					((GraphDefinedDomain) dg).setTransition(currentStateIndex, 2/*down action*/, resultingStateIndex, downHM.get(resultingStateIndex));
				}

				//Up
				HashMap<Integer, Double> upHM = new HashMap<Integer, Double>();
				addToExistingValue(upHM, aboveIndex, 1.0-slipProbability);
				addToExistingValue(upHM, rightIndex, slipProbability/2.0);
				addToExistingValue(upHM, leftIndex, slipProbability/2.0);
				for (Integer resultingStateIndex : upHM.keySet()) {
					((GraphDefinedDomain) dg).setTransition(currentStateIndex, 3/*up action*/, resultingStateIndex, upHM.get(resultingStateIndex));
				}
			}
		}

			return dg;
		}	

		private static void addToExistingValue(HashMap<Integer, Double> hm, Integer key, Double prob) {
			if (hm.containsKey(key)) {
				Double oldVal = hm.get(key);
				hm.put(key, oldVal+prob);
			}
			else {
				hm.put(key, prob);
			}
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
