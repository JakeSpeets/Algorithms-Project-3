# -*- coding: utf-8 -*-
"""
This project implements and tests graph algorithms to solve the following problems:
1.
- Use BFS and DFS to find all connected components in an undirected graph.
- Use BFS and DFS to find the shortest path between two nodes in an undirected graph.
2.
- Find the strongly connected components in a directed graph.
- Derive a directed graph as a meta graph of the strongly connected components of a directed graph,
  then represent the meta graph as a directed acyclic graph and linearize it.
3.
- Use Dijkstra's algorithm to produce the shortest path tree for a weighted directed graph.
- Produce a minimum spanning tree for a weighted directed graph using Prim's algorithm.

The project uses the NetworkX library to represent graphs.
The following graph algorithms will be implemented in algos.py:
- Breadth-first search (BFS)
- Depth-first search (DFS)
- Dijkstra's algorithm
- Prim's algorithm

"""

import networkx as nx
import algos
import scc

def main():
    
    # Creating an undirected graph
    A = "A"
    B = "B"
    F = "F"
    E = "E"
    C = "C"
    D = "D"
    G = "G"
    J = "J"
    I = "I"
    M = "M"
    N = "N"
    H = "H"
    K = "K"
    L = "L"
    O = "O"
    P = "P"

    graph = nx.Graph()
    graph.add_edge(A, B)
    graph.add_edge(A, F)
    graph.add_edge(A, E)
    graph.add_edge(B, C)
    graph.add_edge(B, F)
    graph.add_edge(C, D)
    graph.add_edge(C, G)
    graph.add_edge(D, G)
    graph.add_edge(E, F)
    graph.add_edge(G, J)
    graph.add_edge(E, I)
    graph.add_edge(F, I)
    graph.add_edge(I, J)
    graph.add_edge(I, M)
    graph.add_edge(M, N)
    graph.add_edge(H, K)
    graph.add_edge(H, L)
    graph.add_edge(K, L)
    graph.add_edge(K, O)
    graph.add_edge(L, P)
    print("Undirected graph created:")

    # Using DFS to find all connected components in the graph
    DFScc = dfs_connected_components(graph)
    print("\nConnected components found using DFS:")
    for component in DFScc:
        print(component)
    print("Number of connected components found using DFS:", len(DFScc))

    # Using BFS to find all connected components in the graph
    BFScc = bfs_connected_components(graph)
    print("\nConnected components found using BFS:")
    for component in BFScc:
        print(component)
    print("Number of connected components found using BFS:", len(BFScc))

    # Using DFS to find the shortest path between two nodes in the graph
    DFSpath = algos.dfs_path(graph, A, F)
    print("\nShortest path found using DFS:", DFSpath)

    # Using BFS to find the shortest path between two nodes in the graph
    BFSpath = algos.bfs_path(graph, A, F)
    print("Shortest path found using BFS:", BFSpath)

    # Creating a directed graph
    graph = nx.DiGraph()
    graph.add_edge(1, 3)
    graph.add_edge(3, 2)
    graph.add_edge(2, 1)
    graph.add_edge(3, 5)
    graph.add_edge(4, 1)
    graph.add_edge(4, 2)
    graph.add_edge(4, 12)
    graph.add_edge(5, 6)
    graph.add_edge(5, 8)
    graph.add_edge(6, 8)
    graph.add_edge(6, 7)
    graph.add_edge(6, 10)
    graph.add_edge(7, 10)
    graph.add_edge(8, 9)
    graph.add_edge(8, 10)
    graph.add_edge(9, 11)
    graph.add_edge(9, 5)
    graph.add_edge(10, 9)
    graph.add_edge(10, 11)
    graph.add_edge(11, 12)
    print("\nDirected graph created:")

    # Finding the strongly connected components in the directed graph
    count, components = scc.scc(graph)
    print("\nStrongly connected components found:")
    for key in components:
        print(components[key])
    print("Number of nodes in each group:", count)
        
    # Creating a meta-graph of strongly connected components
    meta_graph, scc_map = create_meta_graph(graph, components)
    print("\nNodes representing each strongly connected component:")
    print(scc_map)
    print("Meta-graph of strongly connected components:")
    print(meta_graph.edges())
    
    # Linearize the directed acyclic graph
    linearized = algos.dfs_tpl_order(meta_graph, 4, [])
    print("\nLinearized directed acyclic graph:")
    print(linearized)
        
    # Creating a weighted undirected graph
    graph = nx.Graph()
    graph.add_edge('A', 'B', weight=22)
    graph.add_edge('A', 'C', weight=9)
    graph.add_edge('A', 'D', weight=12)
    graph.add_edge('B', 'C', weight=35)
    graph.add_edge('C', 'D', weight=4)
    graph.add_edge('B', 'H', weight=34)
    graph.add_edge('B', 'F', weight=36)
    graph.add_edge('F', 'H', weight=24)
    graph.add_edge('C', 'F', weight=25)
    graph.add_edge('C', 'E', weight=65)
    graph.add_edge('D', 'E', weight=33)
    graph.add_edge('E', 'G', weight=23)
    graph.add_edge('E', 'F', weight=18)
    graph.add_edge('D', 'I', weight=30)
    graph.add_edge('F', 'G', weight=39)
    graph.add_edge('G', 'I', weight=21)
    graph.add_edge('H', 'I', weight=19)
    graph.add_edge('G', 'H', weight=25)
    
    print("\nWeighted directed graph created:")
        
    # Using Dijkstra's algorithm to find the shortest path tree
    shortest_path_tree = algos.dijkstra(graph, 'A')
    print("\nShortest path tree found using Dijkstra's algorithm:")
    print(shortest_path_tree.edges())
        
    # Using Prim's algorithm to find the minimum spanning tree
    mst = algos.prim(graph, 'F')
    print("\nMinimum spanning tree found using Prim's algorithm:")
    print(mst.edges())


def dfs_connected_components(graph):
    # Create a set of all vertices in the graph
    vertices = set(graph.nodes)
    connected_components = []
    # While there are still vertices to visit:
    # for a vertex in vertices, perform dfs on the vertex to obtain the connected component
    while vertices:
        vertex = vertices.pop()
        component = algos.dfs(graph, vertex)
        connected_components.append(component)
        # Remove the vertices in the connected component from the set of vertices
        vertices -= component
    return connected_components

def bfs_connected_components(graph):
    # Create a set of all vertices in the graph
    vertices = set(graph.nodes)
    connected_components = []
    # While there are still vertices to visit:
    # for a vertex in vertices, perform bfs on the vertex to obtain the connected component
    while vertices:
        vertex = vertices.pop()
        component = algos.bfs(graph, vertex)
        connected_components.append(component)
        # Remove the vertices in the connected component from the set of vertices
        vertices -= component
    return connected_components

def create_meta_graph(graph, components):
    meta_graph = nx.DiGraph()
    component_map = {node: i for i, component in components.items() for node in component}
    scc_map = {i: list(component) for i, component in components.items()}

    for node in component_map:
        meta_graph.add_node(component_map[node])

    for edge in graph.edges():
        source_component = component_map[edge[0]]
        target_component = component_map[edge[1]]
        if source_component != target_component:
            meta_graph.add_edge(source_component, target_component)

    return meta_graph, scc_map

main()