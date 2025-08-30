class TreeNode:
    def __init__(self, value, parent=None):
        self.value = value
        self.parent = parent
        self.children = []
    
    def add_child(self, child_node):
        child_node.parent = self
        self.children.append(child_node)
    
    def __repr__(self):
        return f"Node({self.value})"

class ExploringAgent:
    def __init__(self, id, starting_node, true_map, all_responsibilities, taken_responsibilities, responsibility=None, root=0):
        self.id = id
        self.root = root
        self.current_node = starting_node
        self.node_map = {}
        self.finished_exploring = []
        self.true_map = true_map
        self.all_responsibilities = all_responsibilities
        self.taken_responsibilities = taken_responsibilities
        self.update_map()
        self.node_map[self.root].update({'responsibilities': all_responsibilities.copy()})
        self.intercept_responsibility = None
        self.responsibility = responsibility if responsibility else self.id
        self.all_intercepts = []
    
    def visit_node(self, node):
        """Visit a node and update the map with its structure"""
        node = self.true_map[node]
        current_entry = self.node_map.get(node.value, {'visited': False})
        if not current_entry['visited']:
            current_entry.update({
                'parent': node.parent.value if node.parent else None,
                'children': [child.value for child in node.children],
                'visited': True
            })
            self.node_map[node.value] = current_entry
    
    def current_children(self):
        return self.node_map[self.current_node]['children']
    
    def current_parent(self):
        return self.node_map[self.current_node].get('parent', None)
    
    def assign_responsibilities(self):
        if self.node_map[self.current_node]['parent'] is None:
            agents = self.all_responsibilities
        else:
            # Non-root node case: get responsibilities from current node
            agents = self.node_map[self.current_node]['responsibilities']
        
        # Calculate responsibilities for children
        responsibilities = self.calculate_responsibilities(self.current_node, agents)
        
        # Save responsibilities in node_map for each child
        for child_value in self.node_map[self.current_node]['children']:
            if child_value not in self.node_map:
                self.node_map[child_value] = {
                    'responsibilities': responsibilities[child_value],
                    'visited': False
                }
            else:
                # Update the child's responsibilities in the node_map
                self.node_map[child_value]['responsibilities'] = responsibilities[child_value]
    
    def _is_subtree_fully_explored(self, node_value, responsibility):
        """
        Check if all nodes in the subtree rooted at node_value that are this agent's 
        responsibility have been visited.
        """
        if node_value not in self.node_map:
            return False
            
        node_info = self.node_map[node_value]
        
        # Check if this node is our responsibility and if it's been visited
        responsibilities = node_info.get('responsibilities', [])
        if responsibility not in responsibilities:
            return True

        if not node_info.get('visited', False):
            return False
        
        # Recursively check all children
        for child_value in node_info.get('children', []):
            if not self._is_subtree_fully_explored(child_value, responsibility):
                return False
            
        return True
    
    def step(self):
        if self.responsibility is not None:
            return self.open_explore_step()
        elif self.intercept_responsibility is not None:
            return self.intercept_step()
        return self.current_parent()

    def open_explore_step(self):
        self.assign_responsibilities()

        for child_value in self.current_children():
            if not self._is_subtree_fully_explored(child_value, self.responsibility):
                return child_value
        if not self._is_subtree_fully_explored(self.root, self.responsibility):
            return self.current_parent()
        
        self.finished_exploring.append(self.responsibility)
        self.responsibility = None
        return self.step()

    def intercept_step(self):
        self.assign_responsibilities()

        for child_value in self.current_children()[::-1]:
            if not self._is_subtree_fully_explored(child_value, self.intercept_responsibility):
                return child_value
        if not self._is_subtree_fully_explored(self.root, self.intercept_responsibility):
            return self.current_parent()
        
        self.intercept_responsibility = None
        return self.step()
    
    def update_map(self):
        self.visit_node(self.current_node)
    
    def calculate_responsibilities(self, node, agents):
        """
        Distribute agents across the children of a given node.
        
        Args:
            node: TreeNode whose children will be assigned to agents
            agents: List of ExploringAgent instances to distribute
        
        Returns:
            dict: Mapping of child nodes to lists of assigned agents
        """

        children = self.node_map[node].get('children', None)
        if not children:
            return {}
        
        responsibilities = {child: [] for child in children}
        n_agents = len(agents)
        n_children = len(children)

        if n_agents >= n_children:
            agents_per_child = n_agents // n_children
            extra_agents = n_agents % n_children

            remaining_agents = agents.copy()
            for i, child in enumerate(children):
                n_assigned = agents_per_child + (1 if i < extra_agents else 0)
                responsibilities[child].extend(remaining_agents[:n_assigned])
                remaining_agents = remaining_agents[n_assigned:]
        else:
            children_per_agent = n_children // n_agents
            extra_children = n_children % n_agents
            
            remaining_children = children.copy()
            for i, agent in enumerate(agents):
                n_assigned = children_per_agent + (1 if i < extra_children else 0)
                for child in remaining_children[:n_assigned]:
                    responsibilities[child].append(agent)
                remaining_children = remaining_children[n_assigned:]

        return responsibilities