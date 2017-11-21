import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

/**
 * Your implementation of various different graph algorithms.
 *
 * @author YOUR NAME HERE
 * @userid YOUR USER ID HERE(i.e. gburdell3)
 * @GTID YOUR GT ID HERE (i.e. 900000000)
 * @version 1.0
 */
public class GraphAlgorithms {

    /**
     * Performs a breadth first search (bfs) on the input graph, starting at
     * {@code start} which represents the starting vertex.
     *
     * When exploring a vertex, make sure to explore in the order that the
     * adjacency list returns the neighbors to you. Failure to do so may cause
     * you to lose points.
     *
     * You may import/use {@code java.util.Set}, {@code java.util.List},
     * {@code java.util.Queue}, and any classes that implement the
     * aforementioned interfaces, as long as it is efficient.
     *
     * The only instance of {@code java.util.Map} that you may use is the
     * adjacency list from {@code graph}. DO NOT create new instances of Map
     * for BFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @throws IllegalArgumentException if any input
     *  is null, or if {@code start} doesn't exist in the graph
     * @param <T> the generic typing of the data
     * @param start the vertex to begin the bfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     */
    public static <T> List<Vertex<T>> breadthFirstSearch(Vertex<T> start,
                                            Graph<T> graph) {
        ArrayList<Vertex<T>> returnList = new ArrayList<>();
        System.out.println("arraylist created"); //REMOVE
        Queue<Vertex<T>> queue = new LinkedList<>();
        System.out.println("queue created"); //REMOVE
        Map<Vertex<T>, List<Edge<T>>> adjList = graph.getAdjList();
        System.out.println(adjList); //REMOVE
        returnList.add(start);
        queue.add(start);
        while (!queue.isEmpty()) {
            System.out.println("the queue is size " + queue.size()); // REMOVE
            Vertex<T> top = queue.poll();
            List<Edge<T>> adj = adjList.get(top);
            for (Edge<T> edge : adj) {
                if (!returnList.contains(edge.getV())) {
                    System.out.println("entered if statement"); //REMOVE
                    Vertex<T> insertV = edge.getV();
                    returnList.add(insertV);
                    queue.add(insertV);
                    System.out.println("The queue is now size " + queue.size() ); //REMOVE
                }
            }
        }
        return returnList;
    }
    private static <T> List<Vertex<T>> dfsVisit(Vertex<T> start,
                                                Graph<T> graph, ArrayList<Vertex<T>> visitList) {
        List<Edge<T>> adjList = graph.getAdjList().get(start);
        if (!visitList.contains(start)) {
            visitList.add(start);
        }
        for (Edge<T> edge : adjList) {
            if (!visitList.contains(edge.getV())) {
                visitList.add(edge.getV());
                dfsVisit(edge.getV(), graph, visitList);
            }
        }
        return visitList;
    }

    /**
     * Performs a depth first search (dfs) on the input graph, starting at
     * {@code start} which represents the starting vertex.
     *
     * When deciding which neighbors to visit next from a vertex, visit the
     * vertices in the order presented in that entry of the adjacency list.
     *
     * *NOTE* You MUST implement this method recursively, or else you will lose
     * most if not all points for this method.
     *
     * You may import/use {@code java.util.Set}, {@code java.util.List}, and
     * any classes that implement the aforementioned interfaces, as long as it
     * is efficient.
     *
     * The only instance of {@code java.util.Map} that you may use is the
     * adjacency list from {@code graph}. DO NOT create new instances of Map
     * for DFS (storing the adjacency list in a variable is fine).
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @throws IllegalArgumentException if any input
     *  is null, or if {@code start} doesn't exist in the graph
     * @param <T> the generic typing of the data
     * @param start the vertex to begin the dfs on
     * @param graph the graph to search through
     * @return list of vertices in visited order
     */
    public static <T> List<Vertex<T>> depthFirstSearch(Vertex<T> start,
                                            Graph<T> graph) {
        ArrayList<Vertex<T>> returnList = new ArrayList<>();
        return dfsVisit(start, graph, returnList);
    }


    /**
     * Finds the single-source shortest distance between the start vertex and
     * all vertices given a weighted graph (you may assume non-negative edge
     * weights).
     *
     * Return a map of the shortest distances such that the key of each entry
     * is a node in the graph and the value for the key is the shortest distance
     * to that node from start, or Integer.MAX_VALUE (representing infinity)
     * if no path exists.
     *
     * You may import/use {@code java.util.PriorityQueue},
     * {@code java.util.Map}, and {@code java.util.Set} and any class that
     * implements the aforementioned interfaces, as long as it's efficient.
     *
     * You should implement the version of Dijkstra's where you terminate the
     * algorithm once either all vertices have been visited or the PQ becomes
     * empty.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @throws IllegalArgumentException if any input is null, or if start
     *  doesn't exist in the graph.
     * @param <T> the generic typing of the data
     * @param start index representing which vertex to start at (source)
     * @param graph the graph we are applying Dijkstra's to
     * @return a map of the shortest distances from start to every other node
     *         in the graph
     */
    public static <T> Map<Vertex<T>, Integer> dijkstras(Vertex<T> start,
                                                      Graph<T> graph) {
        Map<Vertex<T>, Integer> distances = new HashMap<>();
        Map<Vertex<T>, List<Edge<T>>> adjList = graph.getAdjList();
        //Set<T> edges = (Set<T>) graph.getEdges();
        PriorityQueue<Edge<T>> queue = new PriorityQueue<>();
        Set<Vertex<T>> visited = new HashSet<>();
        visited.add(start);
        for (Vertex<T> vertex : adjList.keySet()) { //create the distance map
            if (!distances.containsKey(vertex) && vertex.getData() == start.getData()) {
                distances.put(vertex, 0);
            } else {
                distances.put(vertex, Integer.MAX_VALUE);
            }
        }
        List<Edge<T>> startAdjEdges = graph.getAdjList().get(start);
        queue.addAll(startAdjEdges);
        System.out.println(distances); //remove
        while (!queue.isEmpty()) {
            Edge<T> minEdge = queue.poll();
            Vertex<T> v1 = minEdge.getU();
            Vertex<T> v2 = minEdge.getV();
            visited.add(v1);
            if (distances.get(v2) > distances.get(v1) + minEdge.getWeight()) {
                distances.put(v2, distances.get(v1) + minEdge.getWeight());
            }
            List<Edge<T>> v2Adjedges = graph.getAdjList().get(v2);
            for (Edge<T> edge : v2Adjedges) {
                Vertex<T> x2 = edge.getV();
                Vertex<T> x1 = edge.getU();
                if (distances.get(x2) > distances.get(x1) + edge.getWeight()) {
                    distances.put(x2, distances.get(x1) + edge.getWeight());
                }
                if (!visited.contains(edge.getV())) {
                    queue.add(edge);
                }
            }
        }
        System.out.println(distances);// ensures that the distances are correct REMOVE
        return distances;
    }


    /**
     * Runs Kruskal's algorithm on the given graph and return the Minimal
     * Spanning Tree (MST) in the form of a set of Edges. If the graph is
     * disconnected and therefore no valid MST, return null.
     *
     * You may assume that the passed in graph is undirected. In this framework,
     * this means that if (u, v, 3) is in the graph, then the opposite edge
     * (v, u, 3) will also be in the graph, though as a separate Edge object.
     *
     * The returned set of edges should form an undirected graph. This means
     * that every time you add an edge to your return set, you should add the
     * opposite edge to the set as well. This is for testing purposes.
     *
     * You may assume that there will only be one valid MST that can be formed.
     *
     * Kruskal's will also require you to use a Disjoint Set which has been
     * provided for you. A Disjoint Set will keep track of which vertices are
     * connected given the edges in your current MST, allowing you to easily
     * figure out whether adding an edge will create a cycle. Refer
     * to the {@code DisjointSet} and {@code DisjointSetNode} classes that
     * have been provided to you for more information.
     *
     * You should NOT allow self-loops into the MST.
     *
     * You may import/use {@code java.util.PriorityQueue},
     * {@code java.util.Set}, and any class that implements the aforementioned
     * interface.
     *
     * DO NOT modify the structure of the graph. The graph should be unmodified
     * after this method terminates.
     *
     * @throws IllegalArgumentException if any input is null
     * @param <T> the generic typing of the data
     * @param graph the graph we are applying Kruskals to
     * @return the MST of the graph or null if there is no valid MST
     */
    public static <T> Set<Edge<T>> kruskals(Graph<T> graph) {
        PriorityQueue<Edge<T>> priQueue = new PriorityQueue<>(graph.getEdges());
        Set<Edge<T>> edges = graph.getEdges();
        List<Vertex<T>> vertexList = new ArrayList<>();
        Set<Edge<T>> mst = new HashSet<>();
        for (Edge<T> edge : edges) { //places verteces into a list
            Vertex<T> v1 = edge.getU();
            if (!vertexList.contains(v1)) {
                vertexList.add(v1);
            }
        }
        DisjointSet<Vertex<T>> disjoint = new DisjointSet<>(vertexList); //creates a disjoint set based on the list created above
        while (!priQueue.isEmpty()) {
            Edge<T> edge = priQueue.poll();
            Vertex<T> u = edge.getU();
            Vertex<T> v = edge.getV();
            Vertex<T> u1 = disjoint.find(u);
            Vertex<T> v1 = disjoint.find(v);
            if (!v1.getData().equals(u1.getData())) {
                mst.add(edge);
                Vertex<T> u2 = edge.getU();
                Vertex<T> v2 = edge.getV();
                Edge<T> reverse = new Edge<>(v2, u2, edge.getWeight());
                mst.add(reverse);
                disjoint.union(u1, v1);
            }
        }

        return mst;


    }
}