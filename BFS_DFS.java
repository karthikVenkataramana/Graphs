/*
Author: Karthik Venkataramana Pemmaraju
Date: 08/07/2017
Description: DFS, BFS, Topological sort and Shortest path in unweighted graph(using BFS) is demonstrated below.
*/

import java.util.*;
/*
  A graph is built of nodes. which have below fields.
*/
class Graph{
  int V;
  int E;
  List<Node> nodes;
  public Graph(int V, int E){
    this.V = V;
    this.E = E;
    nodes = new ArrayList<Node>();
  }
}

class Node{
  char name;
  boolean isVisited;
  int indegree;
  List<Node> outgoing_nodes; // Connection of each nodes.
  protected Node(char name){
    this.name = name;
    this.outgoing_nodes = new ArrayList<Node>();
    this.isVisited = false;
    this.indegree = 0;
  }
  // We say two nodes are equal, if they bear the same name.
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


public class BFS_DFS{

  // Establishes a two - way connection between a to b.
  protected static void setEdge(Node a, Node b){
    a.outgoing_nodes.add(b);
    b.indegree++;
    b.outgoing_nodes.add(a);
    a.indegree++;
  }
  protected static void DFS(Node start){
    // logic is to use Stack. (We get that by recursion.) Backtracking. Similar to preorder traversal.
    if(!start.isVisited)
    System.out.print(start.name + "   ");
    
    start.isVisited = true;
    
    for(Node node: start.outgoing_nodes){
      if(!node.isVisited)
        DFS(node);
        node.isVisited = true;
    } // End of for loop.
    return;
  }

  protected static void BFS(Node start){
    // We need to use a queue. Similar to level order traversal.
    Queue<Node> queue = new LinkedList<Node>();
    queue.add(start);
    start.isVisited = false; // Missed this statement, first - time.

    while(!queue.isEmpty()){
      Node node = queue.poll(); // Returns head value and removes it.
      for(Node out_nodes: node.outgoing_nodes){
        if(out_nodes.isVisited) // Initially, isVisited is false, for DFS we set it to TRUE. So, for BFS we do the reverse process.
          queue.add(out_nodes);
        out_nodes.isVisited = false;
      }
    } // End of While loop.
  }

  protected static  void topologicalSort(Graph graph){
    // Logic is to first get all vertices with in-degree = 0. Then, add adjacent vertices to queue, decrement their degree.
    // No need to check is Visited flag here, Why? nodes with in-degree = 0 have no outgoing nodes which we are looping.
    Queue<Node> queue = new LinkedList<Node>();

    // Initialize queue with nodes whose indegree is 0.
    for(Node node: graph.nodes){
      if(node.indegree == 0)
        queue.add(node);
    }
    if(queue.isEmpty())
      System.out.println("Graph has cycles. Please uncomment in setEdge method.");

    // Add adjacent nodes to queue and decrement their indegree.
    while(!queue.isEmpty()){
      Node node = queue.poll();
      System.out.print(node.name + "   ");
      for(Node out_node: node.outgoing_nodes){
        out_node.indegree--;
        if(out_node.indegree == 0)
          queue.add(out_node);
       }
    } // End of While loop.
  }

    // In undirected graph, shortest path is nothing but BFS
    protected void shortestPathsInUndirectedGraph(Node source){
      int distance = 0;
      boolean loopers = false;

      Queue<Node> queue = new LinkedList<Node>();
      queue.add(source);
      source.isVisited = true; // reverse operation.

      while(!queue.isEmpty()){
        Node node = queue.poll();
        loopers = false;
        for(Node out_node : node.outgoing_nodes){
          if(!out_node.isVisited){
              queue.add(out_node);
              loopers = true;
              System.out.println("DISTANCE FROM " + source.name + " TO " + out_node.name + " is: " + (distance+1)) ;
          }
          out_node.isVisited = true;
        }
        if(loopers) // Add distance only if loopers flag is set.
          distance++;

      } // End while.
  }


  public static void main(String[] args) {
      BFS_DFS bfs_dfs = new BFS_DFS();
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
      graph.nodes.add(E);
      graph.nodes.add(F);
      graph.nodes.add(G);
      graph.nodes.add(H);

/*
Graph:
    A
    |
    B - H
    |   |
    C - E - G
    |   |
    D   F

*/
      // set up connections.
      setEdge(A,B);
      setEdge(B,C);
      setEdge(B,H);
      setEdge(C,D);
      setEdge(C,E);
      setEdge(E,F);
      setEdge(E,G);
      setEdge(E,H);

      System.out.println("DFS:");
      DFS(A);
      System.out.println("\nBFS:");
      BFS(A);
      System.out.println("\n\nTopological Ordering:");
      topologicalSort(graph);
      System.out.println("\n************************");
      bfs_dfs.shortestPathsInUndirectedGraph(G);
  }
}
