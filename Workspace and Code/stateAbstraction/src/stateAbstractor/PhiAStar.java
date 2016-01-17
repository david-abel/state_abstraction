package stateAbstractor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import SARealGenerators.SAPair;
import SARealGenerators.SARealGenerator;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;

/**
 * Collapses states if the optimal actions are equal, and their Q values are within epsilon.
 */
public class PhiAStar extends StateAbstractor {
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
	public PhiAStar(SARealGenerator SARealGen, double epsilon, List<Action> relevantActions) {
		this.SARealVals = SARealGen.getRealsMap();
		this.eps = epsilon;
		this.relActions = relevantActions;
	}

	public List<Action> getMaxActions(State s) {
		
		List<Action> maxActions = new ArrayList<Action>();
		Double maxActVal = 0.0;
		
		for (Action a : relActions) {
			SAPair sa = new SAPair(s, a);
			Double saVal = this.SARealVals.get(sa);
			
			if (saVal > maxActVal) {
				maxActions.clear();
				maxActions.add(a);
				maxActVal = saVal;
			}
			else if (saVal == maxActVal) {
				maxActions.add(a);
			}
		}
		
		return maxActions;
	}
	
	@Override
	protected boolean phi(State s1, State s2) {
		Double maxDiff = 0.0;
		
		// Find their max actions.
		List<Action> s1MaxActions = getMaxActions(s1);
		List<Action> s2MaxActions = getMaxActions(s2);
		
		
		// Loop over optimal actions. If they share an optimal action, and their Q* is within epsilon, return true.
		for (Action a1 : s1MaxActions) {
			for (Action a2 : s2MaxActions) {
				
				if (a1.equals(a2)) {
					Double sa1Val = this.SARealVals.get(new SAPair(s1,a1));
					Double sa2Val = this.SARealVals.get(new SAPair(s2,a2));
					
					if (Math.abs(sa1Val - sa2Val) < this.eps) {
						return true;
					}
				}
				
			}
		}
		return false;
	}
}
