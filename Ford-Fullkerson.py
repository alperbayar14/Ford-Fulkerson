import time
import numpy as np

class Graph:
    def __init__(self, graph):
        self.graph = graph
        self.ROW = len(graph)
        
    def bfs(self, s, t, parent):
        visited = [False] * self.ROW
        queue = []

        queue.append(s)
        visited[s] = True

        while queue:
            u = queue.pop(0)

            for ind, val in enumerate(self.graph[u]):
                if visited[ind] == False and val > 0:
                    queue.append(ind)
                    visited[ind] = True
                    parent[ind] = u

        return True if visited[t] else False

    def ford_fulkerson(self, source, sink):
        parent = [-1] * self.ROW
        max_flow = 0

        while self.bfs(source, sink, parent):
            path_flow = float("Inf")
            s = sink

            while s != source:
                path_flow = min(path_flow, self.graph[parent[s]][s])
                s = parent[s]

            max_flow += path_flow

            v = sink
            while v != source:
                u = parent[v]
                self.graph[u][v] -= path_flow
                self.graph[v][u] += path_flow
                v = parent[v]

        return max_flow

def read_graph_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
    num_nodes = int(lines[0].strip())
    graph = [[0] * num_nodes for _ in range(num_nodes)]
    
    for line in lines[1:]:
        u, v, capacity = map(int, line.strip().split())
        graph[u][v] = capacity
        
    return graph

def measure_execution_time(graph):
    g = Graph(graph)
    source = 0
    sink = len(graph) - 1
    start_time = time.time()
    g.ford_fulkerson(source, sink)
    end_time = time.time()
    execution_time = (end_time - start_time) * 1e9  # Convert to nanoseconds
    return execution_time

def analyze_performance(file_paths):
    results = []

    for file_path in file_paths:
        graph = read_graph_from_file(file_path)
        execution_times = []
        for _ in range(10):  # Run each test 10 times for averaging
            execution_time = measure_execution_time(graph)
            execution_times.append(execution_time)

        max_time = np.max(execution_times)
        min_time = np.min(execution_times)
        avg_time = np.mean(execution_times)
        std_dev = np.std(execution_times)

        num_nodes = len(graph)
        results.append({
            'Graph Size': num_nodes,
            'Max Time (ns)': max_time,
            'Min Time (ns)': min_time,
            'Avg Time (ns)': avg_time,
            'Std Dev (ns)': std_dev
        })

    return results

def calculate_theoretical_complexity(graph_sizes):
    complexities = []

    for num_nodes in graph_sizes:
        V = num_nodes
        E = num_nodes * (num_nodes - 1)
        time_complexity = V * (E ** 2)
        space_complexity = V ** 2
        complexities.append({
            'Graph Size': num_nodes,
            'Estimated Time Complexity': time_complexity,
            'Estimated Space Complexity': space_complexity
        })

    return complexities

def print_results(execution_results, theoretical_complexities):
    print("Experimental Results:")
    for result in execution_results:
        print(result)
    
    print("\nTheoretical Complexities:")
    for complexity in theoretical_complexities:
        print(complexity)

file_paths = [f"graph_{size}.txt" for size in [200, 500, 800]]

execution_results = analyze_performance(file_paths)


theoretical_complexities = calculate_theoretical_complexity([200, 500, 800])


print_results(execution_results, theoretical_complexities)
