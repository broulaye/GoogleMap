/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private Set<GeographicPoint> intersection;
	private HashMap<GeographicPoint, MapNode> map;
	private int numVertices;
	private int numEdges;

	private final int DEFAULT_SIZE = 20;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
		intersection = new HashSet<>(DEFAULT_SIZE);
		map = new HashMap<>(DEFAULT_SIZE);
		this.numVertices = 0;
		this.numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return intersection;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if (location == null || this.contains(location)) {
			return false;
		}
		else {
			intersection.add(location);
            MapNode node = new MapNode(location);
			map.put(location, node);
			numVertices++;
			return true;
		}
	}



	private boolean contains(GeographicPoint location) {
		if (location == null) {
			return true;
		}
		else {
			// TODO: Implement contains
			return map.containsKey(location);
		}
	}


	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		Edge edge1 = new Edge(roadName, roadType, length, from, to);
       // Edge edge2 = new Edge(roadName, roadType, length, to, from);

        map.get(from).getEdges().add(edge1);
        //map.get(to).getEdges().add(edge2);
		numEdges++;

		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{

        if (start == null || goal == null) {
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }

        HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
        Queue<MapNode> toExplore = new LinkedList<MapNode>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        toExplore.add(map.get(start));
        boolean found = false;
        while (!toExplore.isEmpty()) {
            MapNode curr = toExplore.remove();
            if (curr == map.get(goal)) {
                found = true;
                break;
            }
            List<Edge> neighbors = curr.getEdges();
            ListIterator<Edge> it = neighbors.listIterator(neighbors.size());
            while (it.hasPrevious()) {
                GeographicPoint next = it.previous().getEnd();
                if (!visited.contains(next)) {
                    visited.add(next);
                    parentMap.put(next, curr.getLocation());
                    toExplore.add(map.get(next));
					nodeSearched.accept(next);
                }
            }
        }

        if (!found) {
            System.out.println("No path exists");
            return null;
        }
        // reconstruct the path
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        GeographicPoint curr = goal;
        while (curr != map.get(start).getLocation()) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }

        path.addFirst(curr);
        return path;


	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}

		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<>();
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		map.get(start).setCurrDist(0);
		toExplore.add(map.get(start));
		boolean found = false;
        int count = 0;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
            System.out.println("removed node: " + curr.getLocation().toString());
            count++;
			if(!visited.contains(curr)) {
				visited.add(curr.getLocation());
			}
			if (curr == map.get(goal)) {
				found = true;
				break;
			}
			List<Edge> neighbors = curr.getEdges();
			ListIterator<Edge> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				Edge currentEdge = it.previous();
				GeographicPoint next = currentEdge.getEnd();
				Double newDist = curr.getCurrDist() + currentEdge.getLength();
				if (!visited.contains(next) || newDist < map.get(next).getCurrDist()) {
					map.get(next).setCurrDist(newDist);
					visited.add(next);
					parentMap.put(next, curr.getLocation());
					toExplore.add(map.get(next));
					nodeSearched.accept(next);
				}
			}
		}

		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		while (curr != map.get(start).getLocation()) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}

        System.out.println("The final count for dijkstra is: " + count);
		path.addFirst(curr);
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
        if (start == null || goal == null) {
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }

        HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
        PriorityQueue<MapNode> toExplore = new PriorityQueue<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        map.get(start).setCurrDist(0);
        map.get(start).setPredictedDist(0);
        toExplore.add(map.get(start));
        boolean found = false;
        int count = 0;
        while (!toExplore.isEmpty()) {
            MapNode curr = toExplore.remove();
            System.out.println("removed node: " + curr.getLocation().toString());
            count++;
            if(!visited.contains(curr)) {
                visited.add(curr.getLocation());
            }
            if (curr == map.get(goal)) {
                found = true;
                break;
            }
            List<Edge> neighbors = curr.getEdges();
            ListIterator<Edge> it = neighbors.listIterator(neighbors.size());
            while (it.hasPrevious()) {
                Edge currentEdge = it.previous();
                GeographicPoint next = currentEdge.getEnd();
                Double newDist = curr.getCurrDist() + currentEdge.getLength();
                Double disToG = newDist + map.get(next).DistTo(goal);
                if (!visited.contains(next) || disToG < map.get(next).getPredictedDist()) {
                    map.get(next).setCurrDist(newDist);
                    map.get(next).setPredictedDist(disToG);
                    visited.add(next);
                    parentMap.put(next, curr.getLocation());
                    toExplore.add(map.get(next));
                    nodeSearched.accept(next);
                }
            }
        }

        if (!found) {
            System.out.println("No path exists");
            return null;
        }
        // reconstruct the path
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        GeographicPoint curr = goal;
        while (curr != map.get(start).getLocation()) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }

        System.out.println("The final count for aStar is: " + count);
        path.addFirst(curr);
        return path;

		
	}

	
	
	public static void main(String[] args)
	{

        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route = theMap.dijkstra(start,end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	    /*
		System.out.print("Making a new intersection...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the intersection...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  

        MapGraph simpleTestMap = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
        List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


        MapGraph testMap = new MapGraph();
        GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

        // A very simple test using real data
        testStart = new GeographicPoint(32.869423, -117.220917);
        testEnd = new GeographicPoint(32.869255, -117.216927);
        System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
        testroute = testMap.dijkstra(testStart,testEnd);
        testroute2 = testMap.aStarSearch(testStart,testEnd);


        // A slightly more complex test using real data
        testStart = new GeographicPoint(32.8674388, -117.2190213);
        testEnd = new GeographicPoint(32.8697828, -117.2244506);
        System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
        testroute = testMap.dijkstra(testStart,testEnd);
        testroute2 = testMap.aStarSearch(testStart,testEnd);

*/


		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.intersection", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.intersection", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the intersection...");
		GraphLoader.loadRoadMap("data/maps/utc.intersection", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
