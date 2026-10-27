from environment import Map
from collections import deque
import heapq
import math
def bfs(env):
    initial_state = env.get_start_state()
    if env.is_goal_state(initial_state):
        yield ("goal", [initial_state])
        return

    to_visit_queue = deque([initial_state])
    parent_record = {initial_state: None}
    yield ("frontier", initial_state)

    while to_visit_queue:
        current_state = to_visit_queue.popleft()
        yield ("expand", current_state)

        for successor, cost in env.get_successors(current_state):
            if successor not in parent_record:
                
                if env.is_goal_state(successor):
                    parent_record[successor] = current_state
                    path = []
                    curr = successor
                    while curr is not None:
                        path.append(curr)
                        curr = parent_record[curr]
                    path.reverse()
                    yield ("goal", path)
                    return
                
                parent_record[successor] = current_state
                yield ("frontier", successor)
                to_visit_queue.append(successor)

    yield ("fail", None)


def dls(env):

    initial_state = env.get_start_state()
    if env.is_goal_state(initial_state):
        yield ("goal", [initial_state])
        return
        
    max_depth_limit = 200
    
    for d_limit in range(0, max_depth_limit):
        yield ("reset_visuals", None)
        
        stack = [(initial_state, [initial_state])]
        visited_depths = {initial_state: 0}
        yield ("frontier", initial_state)

        while stack:
            current_state, path = stack.pop()
            curr_lvl = len(path) - 1
            yield ("expand", current_state)

            if curr_lvl < d_limit:
                for successor, cost in env.get_successors(current_state):
                    new_depth = curr_lvl + 1
                    
                    if env.is_goal_state(successor):
                        yield ("goal", path + [successor])
                        return

                    if successor not in visited_depths or new_depth < visited_depths[successor]:
                        visited_depths[successor] = new_depth
                        yield ("frontier", successor)
                        stack.append((successor, path + [successor]))

    yield ("fail", None)


def bds(env):
    start = env.get_start_state()
    goal = env.get_goal_state()
    if start == goal:
        yield ("goal", [start])
        return

    q_forward = deque([start])
    q_backward = deque([goal])
    f_trace = {start: None}
    b_trace = {goal: None}

    yield ("frontier", start)
    yield ("frontier", goal)

    def build_full_path(intersection_node):

        path_f = []
        curr = intersection_node
        while curr is not None:
            path_f.append(curr)
            curr = f_trace[curr]
        path_f.reverse()
        
        path_b = []
        curr = b_trace[intersection_node]
        while curr is not None:
            path_b.append(curr)
            curr = b_trace[curr]
            
        return path_f + path_b

    while q_forward and q_backward:
        curr_f = q_forward.popleft()
        yield ("expand", curr_f)
        
        if curr_f in b_trace:
            yield ("goal", build_full_path(curr_f))
            return
            
        for succ_f, cost in env.get_successors(curr_f):
            if succ_f not in f_trace:
                f_trace[succ_f] = curr_f
                yield ("frontier", succ_f)
                q_forward.append(succ_f)

        curr_b = q_backward.popleft()
        yield ("expand", curr_b)
        if curr_b in f_trace:
            yield ("goal", build_full_path(curr_b))
            return
            
        for pred_b, cost in env.get_predecessors(curr_b):
            if pred_b not in b_trace:
                b_trace[pred_b] = curr_b
                yield ("frontier", pred_b)
                q_backward.append(pred_b)

    yield ("fail", None)


def astar(env):
    initial_state = env.get_start_state()
    goal_state = env.get_goal_state()

    def heuristic(state):
        return math.hypot(state[0] - goal_state[0], state[1] - goal_state[1])
    

    def heuristic_bonus(state):
        dx = abs(state[0] - goal_state[0])
        dy = abs(state[1] - goal_state[1])
        manhattan = dx + dy
        
        weight = 35.0
        heavy_bias = dx * 10.0 
        
        if (goal_state[0] < initial_state[0] and state[0] > initial_state[0]) or \
           (goal_state[0] > initial_state[0] and state[0] < initial_state[0]):
            heavy_bias += 5000.0
            
      
        initial_dx = abs(initial_state[0] - goal_state[0])
        initial_dy = abs(initial_state[1] - goal_state[1])
        if dx > (initial_dx * 0.85):
           
            if dy < initial_dy:
                y_progress = initial_dy - dy
                heavy_bias += y_progress * 200.0
                
        return (manhattan * weight) + heavy_bias
     
    active_heuristic = heuristic_bonus

    pq = []
    heapq.heappush(pq, (active_heuristic(initial_state), 0, initial_state))
    
    g_costs = {initial_state: 0}
    parent_record = {initial_state: None}
    visited = set()
    yield ("frontier", initial_state)

    while pq:
        f_cost, current_g, current_state = heapq.heappop(pq)
        if current_state in visited:
            continue

        yield ("expand", current_state)
        visited.add(current_state)

        if env.is_goal_state(current_state):
            path = []
            while current_state is not None:
                path.append(current_state)
                current_state = parent_record[current_state]
            path.reverse()
            print("Path Nodes:" ,len(path))
            print("Extended Nodes:" ,len(visited))
            print("ratio:" ,len(visited)/len(path))
            yield ("goal", path)
            return

        for successor, edge_cost in env.get_successors(current_state):
            new_g = current_g + edge_cost
            
            if successor not in g_costs or new_g < g_costs[successor]:
                g_costs[successor] = new_g
                parent_record[successor] = current_state
                
                f_cost = new_g + active_heuristic(successor)
                heapq.heappush(pq, (f_cost, new_g, successor))
                
                if successor not in visited:
                    yield ("frontier", successor)

    yield ("fail", None)


if __name__ == "__main__":
    map = Map(
        search_algorithm=astar, 
        seed=42,
        delay=5
    )
    map.start()



   
   