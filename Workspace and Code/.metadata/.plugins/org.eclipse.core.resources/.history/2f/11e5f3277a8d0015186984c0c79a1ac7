package stateAbstractor;

import java.util.HashMap;
import java.util.List;

import SARealGenerators.SAPair;
import SARealGenerators.SARealGenerator;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;

/**
 * Collapses states based on similarity of real value evaluations of SA pairs
 * by taking in an SARealGenerator. Two states are collapsed if the real values
 * for any given action as determined by the input SARealGenerator are within epsilon
 * of eachother.
 */
public class PhiSAReal extends StateAbstractor {
	SARealGenerator saRealGen;
	HashMap<SAPair, Double> SARealVals; 
	double eps;
	List<Action> relActions;
	
	/**
	 * 
	 * @param SARealGen
	 * @param epsilon
	 * @param relevantActions
	 */
	public PhiSAReal(SARealGenerator SARealGen, double epsilon, List<Action> relevantActions) {
		this.SARealVals = SARealGen.getRealsMap();
		this.eps = epsilon;
		this.relActions = relevantActions;
	}

	@Override
	protected boolean phi(State s1, State s2) {
		Double maxDiff = 0.0;
		for (Action a : relActions) {
			SAPair sa1 = new SAPair(s1, a);
			SAPair sa2 = new SAPair(s2, a);
			double sa1Val = this.SARealVals.get(sa1);
			double sa2Val = this.SARealVals.get(sa2);
			double saDiff = Math.abs(sa1Val - sa2Val);
			maxDiff = Math.max(maxDiff, saDiff);

			if (saDiff > this.eps) {
				return false;
			}
			
		}

		return true;
	}

}
