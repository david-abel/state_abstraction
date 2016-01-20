package domains.taxi;

import java.util.List;

import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.options.DeterministicTerminationOption;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.core.states.State;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;


public class GetPassengerOptionMaker {

	public static boolean isTaxiEmpty(State s) {
		List<ObjectInstance> allPassengers = s.getObjectsOfClass(TaxiDomainGenerator.PASSENGERCLASS);
	
		for(ObjectInstance passenger : allPassengers) {
			boolean inTaxi = passenger.getBooleanValForAttribute(TaxiDomainGenerator.INTAXIATT);
			
			if (inTaxi) {
				// There's a passenger in a car.
				return false;
			}
		}
		return true;
	}
	
	public static Option makeGetPassengerOption(Domain domain) {
		BFS bfs = new BFS(domain, new IsTaxiFullSCT(), new SimpleHashableStateFactory());
		bfs.toggleDebugPrinting(false);
		Policy optionPolicy = new DDPlannerPolicy(bfs);

		//now that we have the parts of our option, instantiate it
		DeterministicTerminationOption option = new DeterministicTerminationOption("getPassengerOption", optionPolicy, new IsTaxiEmptySCT(), new IsTaxiFullSCT());
//		option.setExpectationHashingFactory(new SimpleHashableStateFactory());
		
		return option;
	}
	
	public static void addOptionsToPlanner(Planner planner, List<Option> options) {
		for (Option o : options) {
			planner.addNonDomainReferencedAction(o);
		}
	}

}




// Initiation condition.
class IsTaxiEmptySCT implements StateConditionTest {
	@Override
	public boolean satisfies(State s) {
		return GetPassengerOptionMaker.isTaxiEmpty(s);
	}
}

// Terminal condition.
class IsTaxiFullSCT implements StateConditionTest {

	@Override
	public boolean satisfies(State s) {
		return GetPassengerOptionMaker.isTaxiEmpty(s);
	}
	
}

