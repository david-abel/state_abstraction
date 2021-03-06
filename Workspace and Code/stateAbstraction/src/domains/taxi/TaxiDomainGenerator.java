package domains.taxi;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import burlap.behavior.policy.Policy;
import burlap.behavior.policy.Policy.ActionProb;
import burlap.behavior.singleagent.options.DeterministicTerminationOption;
import burlap.behavior.singleagent.options.Option;
import burlap.oomdp.core.objects.MutableObjectInstance;
import burlap.oomdp.core.objects.ObjectInstance;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.oomdp.core.*;
import burlap.oomdp.core.Attribute.AttributeType;
import burlap.oomdp.core.states.MutableState;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.Action;
import burlap.oomdp.singleagent.FullActionModel;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.singleagent.SADomain;
import burlap.oomdp.singleagent.common.SimpleAction.SimpleDeterministicAction;
import burlap.oomdp.singleagent.explorer.VisualExplorer;
import burlap.oomdp.visualizer.StaticPainter;
import burlap.oomdp.visualizer.Visualizer;

public class TaxiDomainGenerator implements DomainGenerator {

	public static final String								XATT = "xAtt";
	public static final String								YATT = "yAtt";
	public static final String								FUELATT = "fuelAtt";
	public static final String								INTAXIATT = "inTaxiAtt";
	public static final String								WALLMINATT = "wallMinAtt";
	public static final String								WALLMAXATT = "wallMaxAtt";
	public static final String								WALLOFFSETATT = "wallOffsetAtt";

	public static final String								LOCATIONATT = "locationAtt";

	public static final String								BEENPICKEDUPATT = "beenPickedupAtt";


	public static final String								TAXICLASS = "taxi";
	public static final String								LOCATIONCLASS = "location";
	public static final String								PASSENGERCLASS = "passenger";
	public static final String								HWALLCLASS = "horizontalWall";
	public static final String								VWALLCLASS = "verticalWall";


	public static final String								NORTHACTION = "north";
	public static final String								SOUTHACTION = "south";
	public static final String								EASTACTION = "east";
	public static final String								WESTACTION = "west";
	public static final String								PICKUPACTION = "pickup";
	public static final String								DROPOFFACTION = "dropoff";
	public static final String								FILLUPACTION = "fillup";


	public static final String								TAXIATLOCATIONPF = "taxiAt";
	public static final String								PASSENGERATLOCATIONPF = "passengerAt";
	public static final String								TAXIATPASSENGERPF = "taxiAtPassenger";
	public static final String								WALLNORTHPF = "wallNorth";
	public static final String								WALLSOUTHPF = "wallSouth";
	public static final String								WALLEASTPF = "wallEast";
	public static final String								WALLWESTPF = "wallWest";
	public static final String								PASSENGERINTAXI = "inTaxi";



	public static int	minMinX = 5;
	public static int	maxMaxX = 30;
	public static int	minMinY = 5;
	public static int	maxMaxY = 30;
	
	public int												maxX = maxMaxX;
	public int												maxY = maxMaxY;
	public static int												maxFuel = 12;

	public boolean											includeFuel = false;
	public boolean											includePickedup = false;




	@Override
	public Domain generateDomain() {
				
		Domain domain = new SADomain();

		// Attributes.
		Attribute xAtt = new Attribute(domain, XATT, AttributeType.DISC);
		xAtt.setDiscValuesForRange(0, maxX, 1);

		Attribute yAtt = new Attribute(domain, YATT, AttributeType.DISC);
		yAtt.setDiscValuesForRange(0, maxY, 1);

		Attribute fuelAtt = null;;
		if(this.includeFuel){
			fuelAtt = new Attribute(domain, FUELATT, AttributeType.DISC);
			fuelAtt.setDiscValuesForRange(0, maxFuel, 1);
		}

		Attribute inTaxiAtt = new Attribute(domain, INTAXIATT, AttributeType.BOOLEAN);

		Attribute wallMinAtt = new Attribute(domain, WALLMINATT, AttributeType.DISC);
		wallMinAtt.setDiscValuesForRange(0, Math.max(this.maxX, this.maxY)+1, 1);

		Attribute wallMaxAtt = new Attribute(domain, WALLMAXATT, AttributeType.DISC);
		wallMaxAtt.setDiscValuesForRange(0, Math.max(this.maxX, this.maxY)+1, 1);

		Attribute wallOffsetAtt = new Attribute(domain, WALLOFFSETATT, AttributeType.DISC);
		wallOffsetAtt.setDiscValuesForRange(0, Math.max(this.maxX, this.maxY)+1, 1);

		Attribute locationAtt = new Attribute(domain, LOCATIONATT, AttributeType.DISC);
		List<String> locationTypes = new ArrayList<String>();
		locationTypes.add("fuel");
		locationTypes.add("red");
		locationTypes.add("green");
		locationTypes.add("blue");
		locationTypes.add("yellow");
		locationTypes.add("magenta");
		locationTypes.add("pink");
		locationTypes.add("orange");
		locationTypes.add("cyan");
		locationAtt.setDiscValues(locationTypes);

		Attribute beenPickedupAtt = null;
		if(this.includePickedup){
			beenPickedupAtt = new Attribute(domain, BEENPICKEDUPATT, AttributeType.BOOLEAN);
		}

		
		// Objects.
		ObjectClass taxi = new ObjectClass(domain, TAXICLASS);
		taxi.addAttribute(xAtt);
		taxi.addAttribute(yAtt);
		if(this.includeFuel){
			taxi.addAttribute(fuelAtt);
		}

		ObjectClass location = new ObjectClass(domain, LOCATIONCLASS);
		location.addAttribute(xAtt);
		location.addAttribute(yAtt);
		location.addAttribute(locationAtt);

		ObjectClass passenger = new ObjectClass(domain, PASSENGERCLASS);
		passenger.addAttribute(xAtt);
		passenger.addAttribute(yAtt);
		passenger.addAttribute(locationAtt);
		passenger.addAttribute(inTaxiAtt);
		if(this.includePickedup){
			passenger.addAttribute(beenPickedupAtt);
		}

		ObjectClass horizontalWall = new ObjectClass(domain, HWALLCLASS);
		horizontalWall.addAttribute(wallMinAtt);
		horizontalWall.addAttribute(wallMaxAtt);
		horizontalWall.addAttribute(wallOffsetAtt);

		ObjectClass verticalWall = new ObjectClass(domain, VWALLCLASS);
		verticalWall.addAttribute(wallMinAtt);
		verticalWall.addAttribute(wallMaxAtt);
		verticalWall.addAttribute(wallOffsetAtt);


		// Actions.
		new MoveAction(NORTHACTION, domain, 0, 1);
		new MoveAction(SOUTHACTION, domain, 0, -1);
		new MoveAction(EASTACTION, domain, 1, 0);
		new MoveAction(WESTACTION, domain, -1, 0);
		new PickupAction(domain);
		new DropoffAction(domain);
		if(this.includeFuel){
			new FillupAction(domain);
		}
				
		//ADD PFS
		new TaxiDomainGenerator.wallNorth(WALLNORTHPF, domain, "");
		new TaxiDomainGenerator.wallEast(WALLEASTPF, domain, "");
		new TaxiDomainGenerator.wallSouth(WALLSOUTHPF, domain, "");
		new TaxiDomainGenerator.wallWest(WALLWESTPF, domain, "");
		new TaxiDomainGenerator.passengerInTaxi(PASSENGERINTAXI, domain, "");
		new TaxiDomainGenerator.taxiAtPassenger(TAXIATPASSENGERPF, domain, "");

		return domain;
	}

	public static State getRandomState(Domain domain){
		Random rand = new Random();
		int maxX = rand.nextInt(maxMaxX-minMinX) + minMinX;
		int maxY = rand.nextInt(maxMaxX-minMinX) + minMinX;
		
		State s = new MutableState();

		ObjectInstance taxi = new MutableObjectInstance(domain.getObjectClass(TAXICLASS), "taxi");
		s.addObject(taxi);

		boolean usesFuel = domain.getAttribute(FUELATT) != null; 

		if(usesFuel){
			addNInstances(domain, s, LOCATIONCLASS, 5);
		}
		else{
			addNInstances(domain, s, LOCATIONCLASS, 4);
		}

		addNInstances(domain, s, PASSENGERCLASS, 1);
		addNInstances(domain, s, HWALLCLASS, 2);
		addNInstances(domain, s, VWALLCLASS, 5);

		if(usesFuel){
			setTaxi(s, 0, 3, 12);
		}
		else{
			setTaxi(s, rand.nextInt(maxX), rand.nextInt(maxY));
		}

		setLocation(s, 0, rand.nextInt(maxX), rand.nextInt(maxY), 4);
		setLocation(s, 1, rand.nextInt(maxX), rand.nextInt(maxY), 1);
		setLocation(s, 2, rand.nextInt(maxX), rand.nextInt(maxY), 3);
		setLocation(s, 3, rand.nextInt(maxX), rand.nextInt(maxY), 2);

		if(usesFuel){
			setLocation(s, 4, 2, 1, 0);
		}

		setPassenger(s, 0, rand.nextInt(maxX), rand.nextInt(maxY), 1);

		//Walls around edges
		setHWall(s, 0, 0, maxX, maxY);
		setHWall(s, 1, 0, maxX, 0);

		setVWall(s, 0, 0, maxY, maxX);
		setVWall(s, 1, 0, maxY, 0);
		
		//Random walls
		setVWall(s, 2, 0, 2, rand.nextInt(maxX));
		setVWall(s, 3, 3, 5, rand.nextInt(maxX));
		setVWall(s, 4, 0, 2, rand.nextInt(maxX));

		return s;

	}
	
	public State getInitialState(Domain domain){
		State s = new MutableState();

		ObjectInstance taxi = new MutableObjectInstance(domain.getObjectClass(TAXICLASS), "taxi");
		s.addObject(taxi);

		boolean usesFuel = domain.getAttribute(FUELATT) != null; 

		if(usesFuel){
			addNInstances(domain, s, LOCATIONCLASS, 5);
		}
		else{
			addNInstances(domain, s, LOCATIONCLASS, 4);
		}

		addNInstances(domain, s, PASSENGERCLASS, 1);
		addNInstances(domain, s, HWALLCLASS, 2);
		addNInstances(domain, s, VWALLCLASS, 5);

		if(usesFuel){
			setTaxi(s, 0, 3, 12);
		}
		else{
			setTaxi(s, 0, 3);
		}

		setLocation(s, 0, 0, 0, 4);
		setLocation(s, 1, 0, 4, 1);
		setLocation(s, 2, 3, 0, 3);
		setLocation(s, 3, 4, 4, 2);

		if(usesFuel){
			setLocation(s, 4, 2, 1, 0);
		}

		setPassenger(s, 0, 3, 0, 1);

		//Walls around borders
		setHWall(s, 0, 0, 5, 0);
		setHWall(s, 1, 0, 5, 5);

		setVWall(s, 0, 0, 5, 0);
		setVWall(s, 1, 0, 5, 5);
		
		//Random vertical walls
		setVWall(s, 2, 0, 2, 1);
		setVWall(s, 3, 3, 5, 2);
		setVWall(s, 4, 0, 2, 3);

		return s;

	}

	protected static void addNInstances(Domain domain, State s, String className, int n){
		ObjectClass oc = domain.getObjectClass(className);
		for(int i = 0; i < n; i++){
			ObjectInstance o = new MutableObjectInstance(oc, className+i);
			s.addObject(o);
		}
	}


	public static void setTaxi(State s, int x, int y){
		ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
		taxi.setValue(XATT, x);
		taxi.setValue(YATT, y);

	}

	public static void setTaxi(State s, int x, int y, int fuel){
		ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
		setTaxi(s, x, y);
		taxi.setValue(FUELATT, fuel);	
	}

	/**
	 * 
	 * @param s
	 * @param i
	 * @param x
	 * @param y
	 * @param lt
	 */
	public static void setLocation(State s, int i, int x, int y, int lt){
		ObjectInstance l = s.getObjectsOfClass(LOCATIONCLASS).get(i);
		l.setValue(XATT, x);
		l.setValue(YATT, y);
		l.setValue(LOCATIONATT, lt);
	}

	/**
	 * 
	 * @param s
	 * @param i
	 * @param x
	 * @param y
	 * @param lt
	 */
	public static void setPassenger(State s, int i, int x, int y, int lt){
		ObjectInstance p = s.getObjectsOfClass(PASSENGERCLASS).get(i);
		p.setValue(XATT, x);
		p.setValue(YATT, y);
		p.setValue(LOCATIONATT, lt);
		p.setValue(INTAXIATT, 0);
		if(p.getObjectClass().domain.getAction(BEENPICKEDUPATT) != null){
			p.setValue(BEENPICKEDUPATT, 0);
		}

	}

	public static void setHWall(State s, int i, int wallMin, int wallMax, int wallOffset){
		ObjectInstance w = s.getObjectsOfClass(HWALLCLASS).get(i);
		w.setValue(WALLMINATT, wallMin);
		w.setValue(WALLMAXATT, wallMax);
		w.setValue(WALLOFFSETATT, wallOffset);
	}

	public static void setVWall(State s, int i, int wallMin, int wallMax, int wallOffset){
		ObjectInstance w = s.getObjectsOfClass(VWALLCLASS).get(i);
		w.setValue(WALLMINATT, wallMin);
		w.setValue(WALLMAXATT, wallMax);
		w.setValue(WALLOFFSETATT, wallOffset);
	}

	protected State move(State s, int dx, int dy){

		ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
		int tx = taxi.getIntValForAttribute(XATT);
		int ty = taxi.getIntValForAttribute(YATT);

		int nx = tx+dx;
		int ny = ty+dy;

		//using fuel?
		if(this.includeFuel){
			int fuel = taxi.getIntValForAttribute(FUELATT);
			if(fuel == 0){
				//no movement possible
				return s;
			}
			taxi.setValue(FUELATT, fuel-1);
		}



		//check for wall bounding

		if(dx > 0){
			List<ObjectInstance> vwalls = s.getObjectsOfClass(VWALLCLASS);
			for(ObjectInstance wall : vwalls){
				if(wallEast(tx, ty, wall)){
					nx = tx;
					break;
				}
			}
		}
		else if(dx < 0){
			List<ObjectInstance> vwalls = s.getObjectsOfClass(VWALLCLASS);
			for(ObjectInstance wall : vwalls){
				if(wallWest(tx, ty, wall)){
					nx = tx;
					break;
				}
			}
		}
		else if(dy > 0){
			List<ObjectInstance> hwalls = s.getObjectsOfClass(HWALLCLASS);
			for(ObjectInstance wall : hwalls){
				if(wallNorth(tx, ty, wall)){
					ny = ty;
					break;
				}
			}
		}
		else if(dy < 0){
			List<ObjectInstance> hwalls = s.getObjectsOfClass(HWALLCLASS);
			for(ObjectInstance wall : hwalls){
				if(wallSouth(tx, ty, wall)){
					ny = ty;
					break;
				}
			}
		}


		taxi.setValue(XATT, nx);
		taxi.setValue(YATT, ny);


		//do we need to move a passenger as well?
		List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
		for(ObjectInstance p : passengers){
			int inTaxi = p.getIntValForAttribute(INTAXIATT);
			if(inTaxi == 1){
				p.setValue(XATT, nx);
				p.setValue(YATT, ny);
			}
		}

		return s;
	}

	public static class wallEast extends PropositionalFunction {


		public wallEast(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}


		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);


			//check for wall bounding

			List<ObjectInstance> vwalls = s.getObjectsOfClass(VWALLCLASS);
			for(ObjectInstance wall : vwalls){
				if(wallEast(tx, ty, wall)){
					return true;
				}
			}
			return false;
		}

	}

	public static class wallWest extends PropositionalFunction {


		public wallWest(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}


		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);


			//check for wall bounding

			List<ObjectInstance> vwalls = s.getObjectsOfClass(VWALLCLASS);
			for(ObjectInstance wall : vwalls){
				if(wallWest(tx, ty, wall)){
					return true;
				}
			}
			return false;
		}
	}

	public static class wallSouth extends PropositionalFunction {


		public wallSouth(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}


		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);


			//check for wall bounding

			List<ObjectInstance> hwalls = s.getObjectsOfClass(HWALLCLASS);
			for(ObjectInstance wall : hwalls){
				if(wallSouth(tx, ty, wall)){
					return true;
				}
			}
			return false;
		}
	}

	public static class wallNorth extends PropositionalFunction {


		public wallNorth(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}


		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);


			//check for wall bounding

			List<ObjectInstance> hwalls = s.getObjectsOfClass(HWALLCLASS);
			for(ObjectInstance wall : hwalls){
				if(wallNorth(tx, ty, wall)){
					return true;
				}
			}
			return false;
		}

	}

	public static class passengerInTaxi extends PropositionalFunction {

		public passengerInTaxi(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			
			List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
			for (ObjectInstance pass : passengers) {
				if (pass.getIntValForAttribute(INTAXIATT) == 1) return true;
			}
			return false;
		}
	}

	public static class taxiAtPassenger extends PropositionalFunction {

		public taxiAtPassenger(String name, Domain domain, String parameterClasses) {
			super(name, domain, parameterClasses);
		}

		@Override
		public boolean isTrue(State s, String[] params) {
			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int taxiX = taxi.getIntValForAttribute(XATT);
			int taxiY = taxi.getIntValForAttribute(YATT);

			List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
			for (ObjectInstance pass : passengers) {
				int passX = pass.getIntValForAttribute(XATT);
				int passY = pass.getIntValForAttribute(YATT);
				if (taxiX == passX && taxiY == passY) return true;
			}
			return false;
		}
	}


	public class MoveAction extends SimpleDeterministicAction implements FullActionModel {

		int dx;
		int dy;

		public MoveAction(String name, Domain domain, int dx, int dy){
			super(name, domain);
			this.dx = dx;
			this.dy = dy;
		}

		@Override
		protected State performActionHelper(State s,
				GroundedAction groundedAction) {
			return move(s, this.dx, this.dy);
		}
	}


	public class PickupAction extends SimpleDeterministicAction implements FullActionModel {

		public PickupAction(Domain domain){
			super(PICKUPACTION, domain);
		}

		@Override
		protected State performActionHelper(State s,
				GroundedAction groundedAction) {

			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);

			List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
			for(ObjectInstance p : passengers){
				int px = p.getIntValForAttribute(XATT);
				int py = p.getIntValForAttribute(YATT);

				if(tx == px && ty == py){
					p.setValue(INTAXIATT, 1);
					break;
				}

			}

			return s;
		}

	}

	public class DropoffAction extends SimpleDeterministicAction implements FullActionModel {

		public DropoffAction(Domain domain){
			super(DROPOFFACTION, domain);
		}

		@Override
		protected State performActionHelper(State s,
				GroundedAction groundedAction) {

			List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
			for(ObjectInstance p : passengers){
				int in = p.getIntValForAttribute(INTAXIATT);
				if(in == 1){
					p.setValue(INTAXIATT, 0);
					break;
				}
			}

			return s;
		}

	}


	public class FillupAction extends SimpleDeterministicAction implements FullActionModel {

		public FillupAction(Domain domain){
			super(FILLUPACTION, domain);
		}


		@Override
		protected State performActionHelper(State s,
				GroundedAction groundedAction) {

			if(!includeFuel){
				return s;
			}

			ObjectInstance taxi = s.getFirstObjectOfClass(TAXICLASS);
			int tx = taxi.getIntValForAttribute(XATT);
			int ty = taxi.getIntValForAttribute(YATT);

			List<ObjectInstance> locations = s.getObjectsOfClass(LOCATIONCLASS);
			for(ObjectInstance l : locations){
				int lt = l.getIntValForAttribute(LOCATIONATT);
				if(lt == 0){
					int lx = l.getIntValForAttribute(XATT);
					int ly = l.getIntValForAttribute(YATT);
					if(tx == lx && ty == ly){
						taxi.setValue(FUELATT, maxFuel);
					}
				}
			}

			return s;
		}

	}


	public static boolean wallEast(int tx, int ty, ObjectInstance wall){
		int wallo = wall.getIntValForAttribute(WALLOFFSETATT);
		if(wallo == tx+1){
			int wallmin = wall.getIntValForAttribute(WALLMINATT);
			int wallmax = wall.getIntValForAttribute(WALLMAXATT);
			return ty >= wallmin && ty < wallmax;
		}
		return false;
	}

	public static boolean wallWest(int tx, int ty, ObjectInstance wall){
		int wallo = wall.getIntValForAttribute(WALLOFFSETATT);
		if(wallo == tx){
			int wallmin = wall.getIntValForAttribute(WALLMINATT);
			int wallmax = wall.getIntValForAttribute(WALLMAXATT);
			return ty >= wallmin && ty < wallmax;
		}
		return false;
	}


	public static boolean wallNorth(int tx, int ty, ObjectInstance wall){
		int wallo = wall.getIntValForAttribute(WALLOFFSETATT);
		if(wallo == ty+1){
			int wallmin = wall.getIntValForAttribute(WALLMINATT);
			int wallmax = wall.getIntValForAttribute(WALLMAXATT);
			return tx >= wallmin && tx < wallmax;
		}
		return false;
	}

	public static boolean wallSouth(int tx, int ty, ObjectInstance wall){
		int wallo = wall.getIntValForAttribute(WALLOFFSETATT);
		if(wallo == ty){
			int wallmin = wall.getIntValForAttribute(WALLMINATT);
			int wallmax = wall.getIntValForAttribute(WALLMAXATT);
			return tx >= wallmin && tx < wallmax;
		}
		return false;
	}

	public List<Option> getOptions(Domain domain) {
		Option getPassengerOpt = GetPassengerOptionMaker.makeGetPassengerOption(domain);
		Option dropOffPassengerOpt = DropPassengerOffOptionMaker.makeDropOffPassengerOption(domain);

		List<Option> allOptions = new ArrayList<Option>();
		
		allOptions.add(getPassengerOpt);
		allOptions.add(dropOffPassengerOpt);
		
		return allOptions;
	}




	/**
	 * @param args
	 */
	public static void main(String[] args) {

		TaxiDomainGenerator dg = new TaxiDomainGenerator();
		dg.includeFuel = false;
		Domain d = dg.generateDomain();
		State s = TaxiDomainGenerator.getRandomState(d);

		//TerminalExplorer exp = new TerminalExplorer(d);
		//exp.exploreFromState(s);



	}

	
	public static class TaxiRF implements RewardFunction {

		@Override
		public double reward(State s, GroundedAction a, State sprime) {
			List<ObjectInstance> passengers = sprime.getObjectsOfClass(PASSENGERCLASS);
			for (ObjectInstance passenger: passengers) {
				if (passenger.getIntValForAttribute(INTAXIATT) == 1 || !TaxiTF.passengerAtLocation(passenger, sprime)) return 0;
			}
			return 1.0;
		}
		
	}

	public static class TaxiTF implements TerminalFunction {

		private static boolean passengerAtLocation(ObjectInstance passenger, State state) {
			int xLoc = passenger.getIntValForAttribute(XATT);
			int yLoc = passenger.getIntValForAttribute(YATT);
			int agentDestType = passenger.getIntValForAttribute(LOCATIONATT);
			List<ObjectInstance> goalLocations = state.getObjectsOfClass(LOCATIONCLASS);
			for (ObjectInstance goalLoc :goalLocations) {
				int goalX = goalLoc.getIntValForAttribute(XATT);
				int goalY = goalLoc.getIntValForAttribute(YATT);
				if (xLoc == goalX && yLoc == goalY && goalLoc.getIntValForAttribute(LOCATIONATT) == agentDestType) {
					return true;
				}
			}
			return false;
		}

		@Override
		public boolean isTerminal(State s) {
			List<ObjectInstance> passengers = s.getObjectsOfClass(PASSENGERCLASS);
			for (ObjectInstance passenger: passengers) {
				if (passenger.getIntValForAttribute(INTAXIATT) == 1 || !passengerAtLocation(passenger, s)) return false;
			}
			return true;
		}

	}

}
