/*
Author: Karthik Venkataramana Pemmaraju
Date: 08/10/2017
Description: Computes shortest path from a given node to every other node in the graph.
 Doesn't work with negative weights.
*/

import java.util.*;
/*
 So, A graph is a collection of nodes.
 Nodes have connections or edges with weights on them.
 */
class Graph{
  int V;      // No. of vertices
  int E;      // No. of Edges.
  List<Node> nodes; 
  Graph(int V, int E){
    this.V = V;
    this.E = E;
    nodes = new ArrayList<Node>();
  }
}
/*
An edge has a source, sink and weight.
*/
class Edge{
  Node source;
  Node sink;
  int weight;
  Edge(Node source, Node sink, int weight){
    this.source = source;
    this.sink = sink;
    this.weight = weight;
    }
}

class Node{
  char name;
  int distance;
  boolean isVisited;
  int indegree;
  Node previous;
  List<Edge> outgoing_nodes;
  Node(char name){ // Setting up initial values.
    this.name = name;
    this.isVisited = false;
    this.indegree = 0;
    this.previous = null;
    this.distance = Integer.MAX_VALUE;
    this.outgoing_nodes = new ArrayList<Edge>();
  }
 // We deem two objects two be equal when they have the same name.
  @Override
  public boolean equals(Object object){
    if(object == null)
      return false;
    Node objNode = (Node) object;
    if(this.name == objNode.name)
      return true;
    return false;
  }
}


public class Dijkstra{

  // Establishes a one - way connection between a to b.
  protected static void setEdge(Node a, Node b, int weight){
    Edge edge = new Edge(a, b , weight);
    a.outgoing_nodes.add(edge);
  }
  /*
     Description:  Get's minimum distance node. if it isn't already visited.
      If minimim is Infinite, we return null.
  */
  protected static Node getMinDistance(Graph graph){
    Node minNode = null;
    int min = Integer.MAX_VALUE;
    int distance;

    for(int i = 0; i < graph.nodes.size(); i++){
       Node node = graph.nodes.get(i);
       distance = node.distance;
       if((!node.isVisited) && (distance < min)){
         min = distance;
         minNode = node;
       }
    }

    if(min == Integer.MAX_VALUE)
      minNode = null;

    return minNode;
  }
 /*
  Description: Core logic is implemented here.
 */

  protected static void dijkstra(Graph graph, Node start){
    // Set all distances except for source to infinite. that is done while creating object.
    start.distance = 0;
    Node current = start;
    // Once if a node is visited, we never vist again. Hence total number of iterations will be number of nodes.
    for(int i = 0; i < graph.V; i++){

      for(Edge edge: current.outgoing_nodes){
        int updated_distance = current.distance + edge.weight;
        // update distances  of outgoing nodes.
        if(edge.sink.distance > updated_distance ){
          edge.sink.distance = updated_distance; // Update distance of sink.
          edge.sink.previous = current; // Update previous Node value.
        }

      }
      current.isVisited = true; // Set flag as true;
      // Get minimum of all distances
      Node destination = current;
      //System.out.println(destination.name);

      if(destination.isVisited == true){ // Found the path.
        System.out.println("Shortest distance from " + start.name + " to " + destination.name + " is " + destination.distance);
        System.out.print("PATH: ");
        while(destination != null){
          System.out.print(destination.name + "  ");
          destination = destination.previous;
        }
        System.out.println("\n");
      }

      Node minNode = getMinDistance(graph);
      if(minNode == null){ // No more connections.
        //System.out.println("No such path exists!");
        return;
      }
      current = minNode; // Update current value.
    } // End of for loop.
  }


  public static void main(String[] args) {
    Dijkstra Dijkstra = new Dijkstra();
    Graph graph = new Graph(8, 8); // 8 vertices and 8 edges.

    Node A = new Node('A');
    Node B = new Node('B');
    Node C = new Node('C');
    Node D = new Node('D');
    Node E = new Node('E');
    Node F = new Node('F');
    Node G = new Node('G');
    Node H = new Node('H');

    // Add nodes to the graph
    graph.nodes.add(A);
    graph.nodes.add(B);
    graph.nodes.add(C);
    graph.nodes.add(D);
    graph.nodes.add(E);
    graph.nodes.add(F);
    graph.nodes.add(G);
    graph.nodes.add(H);

/*
Graph:
  A
  | 1
  B -> 2 H
  1|   | 3
  C -> 2 E ->2   G
  4|     |5
  D      F

*/
    // set up connections.
    setEdge(A,B,1);
    setEdge(B,C,1);
    setEdge(B,H,2);
    setEdge(C,D,4);
    setEdge(C,E,2);
    setEdge(E,F,5);
    setEdge(E,G,2);
    setEdge(H,E,3);

    dijkstra(graph, A);
  }
}
