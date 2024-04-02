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

def dijkstra(graph, initial):
    visited = {initial: 0}
    path = {}
    nodes = set(graph.nodes)

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
                if min_node is None:
                    break
                nodes.remove(min_node)
                current_weight = visited[min_node]
                for edge in graph.edges[min_node]:
                    weight = current_weight + graph.distance[(min_node, edge)]
                if edge not in visited or weight < visited[edge]:
                    visited[edge] = weight
                    path[edge] = min_node
    return visited, path

def kruskal(graph):
    mst = nx.Graph()
    edges = list(graph.edges(data=True))
    # Lambda function to sort edges by weight
    edges.sort(key=lambda x: x[2]['weight'])
    # The Functionterates over the sorted list of edges. For each edge, it checks if 
    # adding that edge to the MST would create a cycle.
    for edge in edges:
        if nx.has_path(mst, edge[0], edge[1]):
            continue
        mst.add_edge(edge[0], edge[1], weight=edge[2]['weight'])
    return mst