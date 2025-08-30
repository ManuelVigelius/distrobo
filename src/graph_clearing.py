import pandas as pd
import numpy as np
import random
from collections import namedtuple

def calc_connected_nodes(edges):
    connected_nodes = {}
    all_nodes = np.unique(edges).tolist()
    for node in all_nodes:
        connected = np.unique(edges[(edges == node).any(axis=1)]).tolist()
        connected.remove(node)
        connected_nodes[node] = connected
    return connected_nodes

def calc_new_value(values):
    if len(values) == 0:
        new_score = 1
    elif len(values) == 1:
        new_score = values[0]
    else:
        second_highest, highest = sorted(values)[-2:]
        if highest == second_highest:
            new_score = highest + 1
        else:
            new_score = highest
    return new_score

def calc_scores(connected_nodes):
    scores = {}
    clear_times = {}

    while len(scores) < 2 * (len(connected_nodes) - 1):
        for i, connected in connected_nodes.items():
            existing_results = {}
            existint_times = {}
            missing = None
            continue_flag = False
            for node in connected:
                if (i, node) in scores:
                    existing_results[node] = scores[(i, node)]
                    existint_times[node] = clear_times[(i, node)]
                elif missing is None:
                    missing = node
                else:
                    continue_flag = True
                    break
            
            if continue_flag:
                continue
            
            if missing is not None:
                if (missing, i) not in scores:
                    scores[(missing, i)] = calc_new_value(list(existing_results.values()))
                    if len(existing_results.values()) == 0:
                        clear_times[(missing, i)] = 1
                    else:
                        # should be solution to partition problem without (n_nodes - n)th largest subproblem
                        clear_times[(missing, i)] = sum(existint_times.values()) + 1
            else:
                for node in existing_results.keys():
                    if (node, i) not in scores:
                        tmp = existing_results.copy()
                        del tmp[node]
                        scores[(node, i)] = calc_new_value(list(tmp.values()))
                        tmp2 = existint_times.copy()
                        del tmp2[node]
                        clear_times[(node, i)] = sum(tmp2.values()) + 1
    return scores, clear_times

def calculate_robots_required_for_clear(edges):
    connected = calc_connected_nodes(edges)
    scores, clear_times = calc_scores(connected)

    tmp = []
    for i in range(len(edges)):
        result = 0
        for key in scores.keys():
            if i in key:
                result += scores[key]
        tmp.append(result)

    subtree_clear_costs = pd.DataFrame(np.asarray([
        [key[0] for key in scores.keys()],
        [key[1] for key in scores.keys()],
        [scores[key] for key in scores.keys()],
        # [clear_times[key] for key in scores.keys()]
        ]).T,
        # columns=['from', 'to', 'value', 'time']
        columns=['from', 'to', 'value']
        )
    
    full_tree_clear_cost = pd.merge(left=subtree_clear_costs, left_on=['from', 'to'], right=subtree_clear_costs, right_on=['to', 'from'])
    full_tree_clear_cost['value'] = full_tree_clear_cost[['value_x', 'value_y']].max(axis=1)
    full_tree_clear_cost = full_tree_clear_cost[['from_x', 'value']].rename(columns={'from_x': 'start_node'})
    full_tree_clear_cost = full_tree_clear_cost.groupby('start_node').min().reset_index()

    start_node, required_robots = full_tree_clear_cost.iloc[np.argmin(full_tree_clear_cost['value'])]
    return start_node, required_robots, subtree_clear_costs

def extract_subtree(root, edges):
    processed_nodes = set()
    nodes_to_visit = {root}
    subtree_edges = []

    while len(nodes_to_visit) > 0:
        current = nodes_to_visit.pop()
        processed_nodes.add(current)

        current_edges = edges[np.any(edges == current, axis=1)]
        nodes_to_visit = (nodes_to_visit | set(np.unique(current_edges).tolist())) - processed_nodes
        subtree_edges.append(current_edges)

    return np.unique(np.concat(subtree_edges, axis=0), axis=0)

def split_tree_by_edge(edges, removed_edge):
    remaining = edges[~((edges == removed_edge).all(axis=1) | (edges == removed_edge[::-1]).all(axis=1))]
    return extract_subtree(removed_edge[0], remaining), extract_subtree(removed_edge[1], remaining)

def balanced_edge_cut(edges):    
    n_nodes = len(set(edges.flatten())) + 1
    target_size = n_nodes // 2
    
    def all_connnected(node):
        connected = np.unique(edges[(edges == node).any(axis=1)]).tolist()
        connected.remove(node)
        return connected

    connected_to = {node: all_connnected(node) for node in np.unique(edges).tolist()}

    size_cache = {}
    
    def subtree_size(node, exclude, cache):
        if (node, exclude) in cache:
            return cache[(node, exclude)]
        size = 1 + sum(subtree_size(connected, node, cache) for connected in connected_to[node] if connected != exclude)
        cache[(node, exclude)] = size
        return size
    
    centroid = random.choice(np.unique(edges).tolist())

    while True:
        connected = sorted(connected_to[centroid], key=lambda connected: subtree_size(connected, centroid, size_cache), reverse=True)
        if subtree_size(connected[0], centroid, size_cache) > target_size:
            centroid = connected[0]
        else:
            removed_edge = centroid, connected[0]
            return split_tree_by_edge(edges, removed_edge)
        
Subtree = namedtuple('SubTree', ['edges', 'root', 'cost'])

def split_largest_subtree_until_threshold(tree_edges, cost_threshold):
    subtrees = [Subtree(tree_edges, *calculate_robots_required_for_clear(tree_edges))]
    total_cost = subtrees[0].cost
    unsplittable = []

    if total_cost > cost_threshold:
        return None
    
    while True:
        subtrees.sort(key=lambda subtree: len(subtree.edges))
        largest_subtree = subtrees.pop()
        if len(largest_subtree.edges) <= 1:
            unsplittable.append(largest_subtree)
            break
            
        subtree_edges1, subtree_edges2 = balanced_edge_cut(largest_subtree.edges)
        if (len(subtree_edges1) == 0) or (len(subtree_edges2) == 0):
            unsplittable.append(largest_subtree)
            continue
        subtree1 = Subtree(subtree_edges1, *calculate_robots_required_for_clear(subtree_edges1))
        subtree2 = Subtree(subtree_edges2, *calculate_robots_required_for_clear(subtree_edges2))

        total_cost = total_cost + subtree1.cost + subtree2.cost - largest_subtree.cost + len(subtrees + unsplittable) + 1

        if total_cost > cost_threshold:
            subtrees.append(largest_subtree)
            break
        
        subtrees.extend([subtree1, subtree2])

        if total_cost == cost_threshold:
            break
            
    return [subtree for subtree in subtrees] + unsplittable