# This file contains the implementation of the graph algorithms called in main.py

import networkx as nx


def dfs(graph, start, visited=None):
    """
    Implements a recursive depth-first search
    Sourced from Dr. Hu's lecture notes
    """
    if visited is None:
        visited = set()
    visited.add(start)
    for key in graph.neighbors(start):
        if key not in visited:
            dfs(graph, key, visited)
    return visited

def dfs_path(graph, start, goal):
    """
    Uses DFS to find a path in `graph` between `start` and `goal`.
    Sourced from Dr. Hu's lecture notes
    """
    stack = [(start, [start])]
    visited = set()
    while stack:
        (vertex, path) = stack.pop()
        if vertex not in visited:
            if vertex == goal:
                return path
            visited.add(vertex)
            for neighbor in graph.neighbors(vertex):
                stack.append((neighbor, path + [neighbor]))

def dfs_tpl_order(graph, start, path):
    """
    Returns a topological order of the graph starting from `start` node
    Sourced from Dr. Hu's lecture notes
    """
    path = path + [start]
    # global n
    for edge in graph.neighbors(start):
        if edge not in path:
            path = dfs_tpl_order(graph, edge, path)
    # print(n, start)
    # n -= 1
    return path

def bfs(graph, start):
    """
    Implements a breadth-first search
    """
    visited, queue = set(), [start]
    p = set()
    while queue:
        vertex = queue.pop(0)
        if vertex not in visited:
            visited.add(vertex)
            p.add(vertex)
            queue.extend(set(graph.neighbors(vertex)) - visited)
    return p

def bfs_path(graph, start, goal):
    """
    finds a shortest path in undirected `graph` between `start` and `goal`. 
    If no path is found, returns `None`
    """
    if start == goal:
        return [start]
    visited = {start}
    queue = [(start, [])]

    while queue:
        current, path = queue.pop(0) 
        visited.add(current)
        for neighbor in graph[current]:
            if neighbor == goal:
                return path + [current, neighbor]
            if neighbor in visited:
                continue
            queue.append((neighbor, path + [current]))
            visited.add(neighbor)   
    return None 

def dijkstra(graph, start):
    """
    Uses Dijkstra's algorithm to find the shortest path tree in a weighted graph
    Takes in a networkx graph and a starting node and returns a networkx graph
    """
    shortest_path_tree = nx.Graph()
    shortest_path_tree.add_node(start)
    visited = {start}
    unvisited = set(graph.nodes) - visited
    while unvisited:
        min_edge = None
        min_weight = float('inf')
        for node in visited:
            for neighbor in graph.neighbors(node):
                if neighbor in visited:
                    continue
                weight = graph.get_edge_data(node, neighbor)['weight']
                if weight < min_weight:
                    min_edge = (node, neighbor)
                    min_weight = weight
        if min_edge is None:
            break
        shortest_path_tree.add_edge(min_edge[0], min_edge[1], weight=min_weight)
        visited.add(min_edge[1])
        unvisited.remove(min_edge[1])
    return shortest_path_tree


def prim(graph, start):
    """
    Uses Prim's algorithm to find the minimum spanning tree in a weighted graph
    Takes in a networkx graph and a starting node and returns a networkx graph
    """
    mst = nx.Graph()
    mst.add_node(start)
    visited = {start}
    unvisited = set(graph.nodes) - visited
    while unvisited:
        min_edge = None
        min_weight = float('inf')
        for node in visited:
            for neighbor in graph.neighbors(node):
                if neighbor in visited:
                    continue
                weight = graph.get_edge_data(node, neighbor)['weight']
                if weight < min_weight:
                    min_edge = (node, neighbor)
                    min_weight = weight
        if min_edge is None:
            break
        mst.add_edge(min_edge[0], min_edge[1], weight=min_weight)
        visited.add(min_edge[1])
        unvisited.remove(min_edge[1])
    return mst