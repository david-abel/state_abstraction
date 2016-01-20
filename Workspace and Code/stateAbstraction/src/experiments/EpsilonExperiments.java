package experiments;

import graphStateAbstractionTest.AStarEpsilonTest;
import graphStateAbstractionTest.VIParams;
import graphStateAbstractionTest.EpsilonToNumStatesTuple;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import burlap.behavior.policy.Policy;
import burlap.behavior.policy.RandomPolicy;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.debugtools.DPrint;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.Domain;
import burlap.oomdp.core.TerminalFunction;
import burlap.oomdp.core.states.State;
import burlap.oomdp.singleagent.RewardFunction;
import burlap.oomdp.statehashing.SimpleHashableStateFactory;
import domains.GraphRF;
import domains.NormalDomainToGraphDomain;
import domains.nchain.NChainGenerator;
import domains.randommdp.RandomMDPGenerator;
import domains.taxi.GetPassengerOptionMaker;
import domains.taxi.TaxiDomainGenerator;
import domains.trench.TrenchDomainGenerator;
import domains.upworld.UpWorldGenerator;

/**
 * Code for running experiments for our ICML paper.
 * @author David Abel
 *
 */
public class EpsilonExperiments {
	
	static String filePath = new File("").getAbsolutePath();
	static String resultsDir = filePath + "/src/plotting/";
	
	// Iterate over epsilon and compute the number of states.
	final static double startEpsilon = 0.0;
	final static double endEpsilon = 0.1;
	final static double epsilonIncrement = 0.01;
	
	/**
	 * Given a DomainGenerator + RF + TF + initState, this method generates abstract MDPs subject to different epsilons, and prints the results.
	 * @param domainName
	 * @param gen
	 * @param tf
	 * @param rf
	 * @param initialState
	 */
	public static void generateEpsilonResults(GraphDefinedDomain graphDefinedDomain, RewardFunction graphRF, State initGraphState, String taskName) {
		
//		List<EpsilonToNumStatesTuple> epsilonAndNumStatesPairs = QStarEpsilonTest.testQPhiStateReduction(graphDefinedDomain, graphRF, new NullTermination(), initGraphState, startEpsilon, endEpsilon, epsilonIncrement);
		List<EpsilonToNumStatesTuple> epsilonAndNumStatesPairs = AStarEpsilonTest.testAStarPhiStateReduction(graphDefinedDomain, graphRF, new NullTermination(), initGraphState, startEpsilon, endEpsilon, epsilonIncrement);
		
		List<Double> epsilons = new ArrayList<Double>();
		List<Integer> numStates = new ArrayList<Integer>();
		
		clearOldResultsFile(taskName);
		
		Double randomPolValue = computeValueOfRandomPolicy(graphDefinedDomain, graphRF, initGraphState);
		
		System.out.println("RAND\t" + randomPolValue);
		
		writeDataPointToFile("RAND\t" + randomPolValue.toString(), taskName);
		
		System.out.println("results: ");
		for (EpsilonToNumStatesTuple x : epsilonAndNumStatesPairs) {
			epsilons.add(x.getEpsilon());
			numStates.add(x.getNumStates());
			writeDataPointToFile(x.toString(), taskName);
			System.out.println(x);
		}
		
		// Now make the plot...
	}
	
	private static Double computeValueOfRandomPolicy(GraphDefinedDomain graphDefineddomain, RewardFunction rf, State initState) {
		Domain d = graphDefineddomain.generateDomain();
		Policy randPolc = new RandomPolicy(d);
		
		PolicyIteration PI = new PolicyIteration(d, rf, new NullTermination(), VIParams.gamma, new SimpleHashableStateFactory(), VIParams.maxDelta, VIParams.maxIterations, 1);
		PI.setPolicyToEvaluate(randPolc);
		PI.planFromState(initState);
		
		return PI.value(initState);
	}
	
	private static void clearOldResultsFile(String taskName) {
		FileWriter fileOut;
		try {
			fileOut = new FileWriter(resultsDir + "/" + taskName + "/" + taskName + ".results", false);
			fileOut.write("");
			fileOut.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private static void writeDataPointToFile(String x, String taskName) {
		try(PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(resultsDir + "/" + taskName + "/" + taskName + ".results", true)))) {
			out.println(x);  
		}
		catch (IOException e) {
		    //exception handling left as an exercise for the reader
			e.printStackTrace();
		}
	}
	
	public static void convertAndGenEpsilonResults(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState, String taskName) {
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain();
		Domain d = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(d, rf, tf, graphMaker.graphIndexToNonGraphState, graphMaker.graphActionIndexToNonGraphAction);
		State initGraphState = GraphDefinedDomain.getState(d, graphMaker.initStateID);
		
		// Run experiments.
		generateEpsilonResults(graphDefinedDomain, graphRF, initGraphState, taskName);
	}
	
	public static void convertAndGenEpsilonResultsOptions(DomainGenerator gen, TerminalFunction tf, RewardFunction rf, State initialState, String taskName, List<Option> options) {
		// Convert to graph domain.
		NormalDomainToGraphDomain graphMaker = new NormalDomainToGraphDomain(gen, tf, rf, initialState);
		GraphDefinedDomain graphDefinedDomain = graphMaker.createGraphDomain(options);
		Domain d = graphDefinedDomain.generateDomain();
		RewardFunction graphRF = new GraphRF(d, rf, tf, graphMaker.graphIndexToNonGraphState, graphMaker.graphActionIndexToNonGraphAction);
		State initGraphState = GraphDefinedDomain.getState(d, graphMaker.initStateID);
		
		// Run experiments.
		generateEpsilonResults(graphDefinedDomain, graphRF, initGraphState, taskName);
	}
	
	public static void main(String[] args) {

		DPrint.toggleUniversal(false);
		
		// Create trench domain.
		int height = 3;
		int width = 2;
		TrenchDomainGenerator trenchGen = new TrenchDomainGenerator(height, width);
		TerminalFunction trenchTF = new TrenchDomainGenerator.TrenchTF(height - 1, width - 1);
		RewardFunction trenchRF = new TrenchDomainGenerator.TrenchRF(height - 1, width - 1);
		Domain oldTrenchDomain = trenchGen.generateDomain();
		State initialTrenchState = trenchGen.getInitialState(oldTrenchDomain);
		
		// Create taxi domain.
		TaxiDomainGenerator taxiGen = new TaxiDomainGenerator();
		TerminalFunction taxiTF = new TaxiDomainGenerator.TaxiTF();
		RewardFunction taxiRF = new TaxiDomainGenerator.TaxiRF();
		Domain oldTaxiDomain = taxiGen.generateDomain();
		State initialTaxiState = taxiGen.getInitialState(oldTaxiDomain);
		List<Option> taxiOptions = taxiGen.getOptions(oldTaxiDomain);
		
		// Create upworld domain.
		int upWorldHeight = 10;
		int upWorldWidth = 4;
		GraphDefinedDomain upWorldGen = UpWorldGenerator.getUPWorld(upWorldWidth, upWorldHeight);
		Domain upWorldDomain = upWorldGen.generateDomain();
		State initialUpWorldState = GraphDefinedDomain.getState(upWorldDomain, 0);
		RewardFunction upWorldRF = new UpWorldGenerator.UpWorldRF();
		
		// Create nchain domain.
		int numStates = 50;
		GraphDefinedDomain nChainGen = NChainGenerator.getNStateChain(numStates);
		Domain nChainDomain = nChainGen.generateDomain();
		State initialNChainState = GraphDefinedDomain.getState(nChainDomain, 0);
		RewardFunction nChainRF = new NChainGenerator.nStateChainRF(numStates);
		
		// Create random domain.
		int numRandStates = 1000;

		int numRandActions = 3;
		GraphDefinedDomain randGen = RandomMDPGenerator.getRandomMDP(numRandStates, numRandActions);
		Domain randDomain = randGen.generateDomain();
		State initialRandState = RandomMDPGenerator.getInitialState(randDomain);
		RewardFunction randRF = new RandomMDPGenerator.RandomMDPRF(numRandStates);
		
		String task = "TAXI"; // NCHAIN, TRENCH, TAXI, UPWORLD, RANDOM		
		
		if (task == "ALL") {
			generateEpsilonResults(nChainGen, nChainRF, initialNChainState, "nchain");
			convertAndGenEpsilonResults(trenchGen, trenchTF, trenchRF, initialTrenchState, "trench");
			convertAndGenEpsilonResults(taxiGen, taxiTF, taxiRF, initialTaxiState, "taxi");
			generateEpsilonResults(upWorldGen, upWorldRF, initialUpWorldState, "upworld");
			generateEpsilonResults(randGen, randRF, initialRandState, "random");
		}
		if (task == "NCHAIN") {
			generateEpsilonResults(nChainGen, nChainRF, initialNChainState, "nchain");
		}
		else if (task == "TRENCH") {
			convertAndGenEpsilonResults(trenchGen, trenchTF, trenchRF, initialTrenchState, "trench");
		}
		else if (task == "TAXI") {
			convertAndGenEpsilonResultsOptions(taxiGen, taxiTF, taxiRF, initialTaxiState, "taxi", taxiOptions);
		}
		else if (task == "RANDOM") {
			generateEpsilonResults(randGen, randRF, initialRandState, "random");
		}
		else {
//			TerminalFunction upWorldTF = new UpWorldGenerator.UpWorldTF(upWorldHeight);
			generateEpsilonResults(upWorldGen, upWorldRF, initialUpWorldState, "upworld");
		}
		
		makePlots();
	}
	
	private static void makePlots() {
		String[] cmd = {
		        "/bin/bash",
		        "-c",
		        "python plotter.py all"
		    };
		try {
			Runtime.getRuntime().exec(cmd, null, new File(filePath + "/src/plotting/"));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}