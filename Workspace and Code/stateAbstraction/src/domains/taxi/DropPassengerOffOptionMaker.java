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


public class DropPassengerOffOptionMaker {

	public static boolean isTaxiEmptyAndPassAtDest(State s) {
		List<ObjectInstance> allPassengers = s.getObjectsOfClass(TaxiDomainGenerator.PASSENGERCLASS);
		List<ObjectInstance> allLocations = s.getObjectsOfClass(TaxiDomainGenerator.LOCATIONCLASS);
		
		for(ObjectInstance passenger : allPassengers) {
			boolean inTaxi = passenger.getBooleanValForAttribute(TaxiDomainGenerator.INTAXIATT);

			if (inTaxi) {
				// There's a passenger in a car.
				return false;
			}
			else {
				for (ObjectInstance dest : allLocations) {
					Integer passengerLoc = passenger.getIntValForAttribute(TaxiDomainGenerator.LOCATIONATT);
					Integer destLoc = dest.getIntValForAttribute(TaxiDomainGenerator.LOCATIONATT);
					
					if (passengerLoc == destLoc) {
						Integer passengerX = passenger.getIntValForAttribute(TaxiDomainGenerator.XATT);
						Integer passengerY = passenger.getIntValForAttribute(TaxiDomainGenerator.YATT);
						
						Integer destX = dest.getIntValForAttribute(TaxiDomainGenerator.XATT);
						Integer destY = dest.getIntValForAttribute(TaxiDomainGenerator.YATT);
						
						if (passengerX == destX && passengerY == passengerX) {
							return true;
						}
					}
					
				}
			}
		}
		return false;
	}
	
	public static Option makeDropOffPassengerOption(Domain domain) {
		BFS bfs = new BFS(domain, new IsTaxiFullSCT(), new SimpleHashableStateFactory());
		bfs.toggleDebugPrinting(false);
		Policy optionPolicy = new DDPlannerPolicy(bfs);
		
		//now that we have the parts of our option, instantiate it
		DeterministicTerminationOption option = new DeterministicTerminationOption("dropOffPassengerOption", optionPolicy, new IsTaxiFullSCT(), new IsTaxiEmptyAndPassAtDestSCT());
		option.setExpectationHashingFactory(new SimpleHashableStateFactory());
		
		return option;
	}
	
	public static void addOptionsToPlanner(Planner planner, List<Option> options) {
		for (Option o : options) {
			planner.addNonDomainReferencedAction(o);
		}
	}

}




// Initiation condition.
class IsTaxiEmptyAndPassAtDestSCT implements StateConditionTest {
	@Override
	public boolean satisfies(State s) {
		return DropPassengerOffOptionMaker.isTaxiEmptyAndPassAtDest(s);
	}
}

// Terminal condition.
class IsTaxiFullSCT implements StateConditionTest {

	@Override
	public boolean satisfies(State s) {
		return !GetPassengerOptionMaker.isTaxiEmpty(s);
	}
	
}

