import heapq

UNINITIALIZED = 0
SUCCEEDED     = 2
FAILED        = 3
SEARCHING     = 4

class Node(object):
    def __init__(self, state, parent=None, g=0, h=0):
        self.parent = parent
        self.g, self.h = g, h
        self.state = state

    def __cmp__(self, other):
        return cmp(self.f, other.f)

    @property
    def f(self):
        return self.g + self.h

class AStar(object):

    def __init__(self):
        self._status = UNINITIALIZED
    
    def _reset(self):
        self._open = []
        self._closed = set()

        self._start = None
        self._goal = None

        self._status = UNINITIALIZED
        self.steps = 0

    def initialize(self, start_state, goal_state):
        self._reset()
        self.start = Node(start_state)
        self.goal  = Node(goal_state)
        self.start.g = 0
        self.start.h = start_state.estimate_cost(self.goal.state)
        self.start.parent = None
        
        heapq.heappush(self._open, self.start)

        self._status = SEARCHING

    def search(self):
        while self._status is SEARCHING:
            self.search_step()
            self.steps += 1

        return (self._status == SUCCEEDED)

    def get_solution(self):
        node = self.goal
        solution = [ node ]

        while node.parent is not None:
            solution.append(node.parent)
            node = node.parent

        solution.reverse()

        return solution

    def search_step(self):
        if not self._open:
            self._status = FAILED

        if self._status is not SEARCHING:
            return self._status
        
        node = heapq.heappop(self._open)

        if node.state == self.goal.state:
            self.goal.parent = node.parent
            self._status = SUCCEEDED
        else:
            next_nodes = [ Node(state, parent=node, g=node.g + cost) 
                           for (state, cost) in node.state.successors() ]

            for next in next_nodes:
                cheaper = lambda q: (n for n in q if next.state == n.state 
                                                  and n.g <= next.g)
                if any(cheaper(self._open)) or any(cheaper(self._closed)):
                    continue 

                for nodelist in (self._open, self._closed):
                    worse = [ n for n in nodelist 
                                if next.state == n.state and n.g > next.g ]
                    for n in worse: 
                        nodelist.remove(n)

                heapq.heapify(self._open)

                next.h = next.state.estimate_cost(self.goal.state)
                heapq.heappush(self._open, next)

            self._closed.add(node)

        return self._status