import numpy as np

def synchronize_agent_maps(agents):
    synchronized_map = {}
    
    for agent in agents:
        for node_value, info in agent.node_map.items():
            synced_entry = synchronized_map.get(node_value, info.copy())
            visited = synced_entry.get('visited', False) or info.get('visited', False)
            synced_entry.update(info)
            synced_entry['visited'] = visited
            synchronized_map[node_value] = synced_entry
    
    for agent in agents:
        agent.node_map = synchronized_map.copy()

def synchronize_taken_responsibilities(agents):
    synchronized = list(set(sum([agent.taken_responsibilities for agent in agents], [])))
    for agent in agents:
        agent.taken_responsibilities = synchronized.copy()

def synchronize_finished_exploring(agents):
    synchronized = list(set(sum([agent.finished_exploring for agent in agents], [])))
    for agent in agents:
        agent.finished_exploring = synchronized.copy()

def synchronize_intercepts(agents):
    synchronized = list(set(sum([agent.all_intercepts for agent in agents], [])))
    for agent in agents:
        agent.all_intercepts = synchronized.copy()

def synchronize_clearing_maps(agents):
    nodes_to_clear = set(agents[0].clearing_map.keys())

    for agent in agents:
        nodes_to_clear = nodes_to_clear & {node for node in agent.clearing_map.keys() if not agent.clearing_map[node].get('cleared', False)}

    synchronized_map = agents[0].clearing_map.copy()
    for node in synchronized_map:
        synchronized_map[node]['cleared'] = node in nodes_to_clear

    for agent in agents:
        agent.clearing_map = synchronized_map.copy()

    
def synchronize_information(agents):
    if len(agents) <= 1:
        return
    synchronize_agent_maps(agents)
    synchronize_finished_exploring(agents)
    synchronize_intercepts(agents)
    synchronize_clearing_maps(agents)


def find_groups(agents, edges):
    def is_connected(i, j):
        i = agents[i].current_node
        j = agents[j].current_node
        return (i == j) or np.all([i, j] == edges, axis=1).any() or np.all([j, i] == edges, axis=1).any()

    roots = np.arange(len(agents)).tolist()
    for i in range(0, len(agents)):
        for j in range(i+1, len(agents)):
            if is_connected(i, j):
                new_root = min(roots[i], roots[j])
                roots[i] = new_root
                roots[j] = new_root

    groups = {}
    for agent, root in zip(agents, roots):
        if root in groups:
            groups[root].append(agent)
        else:
            groups[root] = [agent]
    return list(groups.values())

def update_tasks(agents, root):
    list_filter = lambda agent: (agent.responsibility is None) and (agent.intercept_responsibility is None)
    remaining_agents = [agent for agent in agents if list_filter(agent)]
    remaining_agents = sorted(remaining_agents, key=lambda agent: agent.id)
    coordinator = None
    
    for agent in remaining_agents:
        if agent.current_node == root:
            coordinator = agent
            remaining_agents.remove(agent)
            
    if (coordinator is None):
        return        
    
    all_intercepts = coordinator.all_intercepts.copy()
    taken_responsibilities = coordinator.taken_responsibilities.copy()
    open_intercepts = (set(coordinator.all_responsibilities) - set(coordinator.finished_exploring)) - set(coordinator.all_intercepts)
    open_responsibilities = set(coordinator.all_responsibilities) - set(coordinator.taken_responsibilities)
    
    for responsibility in open_responsibilities:
        if len(remaining_agents) == 0:
            break
        remaining_agents.pop().responsibility = responsibility
        taken_responsibilities.append(responsibility)

    for intercept in open_intercepts:
        if len(remaining_agents) == 0:
            break
        remaining_agents.pop().intercept_responsibility = intercept
        all_intercepts.append(intercept)
    
    for agent in agents:
        agent.all_intercepts = all_intercepts.copy()
        agent.taken_responsibilities = taken_responsibilities.copy()

def find_inactive_agents(agents, target_nodes):
    taken_targets = []
    remaining_agents = []
    inactive_positions = {}

    for agent in agents:
        target = target_nodes[agent.id]
        if ((agent.responsibility is None) and (agent.intercept_responsibility is None)) or (target is None):
            inactive_positions[agent.current_node] = agent
            continue
        if target in taken_targets:
            continue
        remaining_agents.append(agent)
        taken_targets.append(target)
    
    return remaining_agents, inactive_positions

def transfer_responsibilities(old_agent, new_agent):
    if old_agent.intercept_responsibility is None:
        tmp = old_agent.responsibility
        old_agent.responsibility = new_agent.responsibility
        new_agent.responsibility = tmp
    else:
        new_agent.intercept_responsibility = old_agent.intercept_responsibility
        old_agent.intercept_responsibility = None
    
def move_active_agents_if_possible(active_agents, blocked_nodes, target_nodes):
    moved_agents = []

    n_remaining = -1
    while len(active_agents) != n_remaining:
        n_remaining = len(active_agents)
        for agent in active_agents.copy():
            if target_nodes[agent.id] not in blocked_nodes.values():
                moved_agents.append(agent)
                del blocked_nodes[agent.id]
                active_agents.remove(agent)
    
    for agent in moved_agents:
        agent.current_node = target_nodes[agent.id]

def move_inactive_agents_if_possible(inactive_agents, all_agents, target_nodes):
    inactive_agents = sorted(inactive_agents, key=lambda agent: agent.id)
    for agent in inactive_agents:
        target = target_nodes[agent.id]
        if target is None:
            continue
        if target not in [agent.current_node for agent in all_agents]:
            agent.current_node = target

def no_collision_move(agents):
    target_nodes = {agent.id: agent.step() for agent in agents}
    agents = sorted(agents, key=lambda agent: agent.id)
    current_nodes = {agent.id: agent.current_node for agent in agents}
    active_agents, inactive_positions = find_inactive_agents(agents, target_nodes)
    
    for agent_iter in active_agents.copy():
        old_agent = agent_iter
        target = target_nodes[old_agent.id]
        while target in inactive_positions:
            new_agent = inactive_positions[target]

            transfer_responsibilities(old_agent, new_agent)
            active_agents.remove(old_agent)
            inactive_positions[old_agent.current_node] = old_agent
            
            new_target = new_agent.step()
            if new_target is None:
                break

            active_agents.append(new_agent)
            del inactive_positions[target]

            old_agent = new_agent
            target_nodes[new_agent.id] = new_target
            target = new_target

    move_active_agents_if_possible(active_agents, current_nodes.copy(), target_nodes)
    move_inactive_agents_if_possible(inactive_positions.values(), agents, target_nodes)