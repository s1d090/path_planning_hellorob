from collections import deque
from heapq import heappush, heappop

from .graph import Cell
from .utils import trace_path


def _is_free_cell(graph, i, j):
    """Return True if the cell is inside the map and not in collision."""
    if not graph.is_cell_in_bounds(i, j):
        return False
    if graph.check_collision(i, j):
        return False
    return True


from collections import deque
from heapq import heappush, heappop

from .graph import Cell
from .utils import trace_path


def _is_free_cell(graph, i, j):
    """Return True if the cell is inside the map and not in collision."""
    if not graph.is_cell_in_bounds(i, j):
        return False
    if graph.check_collision(i, j):
        return False
    return True



def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The grid graph.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    # Reset graph state (visited, parent, g_cost, visited_cells).
    graph.init_graph()

    si, sj = start.i, start.j
    gi, gj = goal.i, goal.j

    # Make sure start and goal are valid.
    if not _is_free_cell(graph, si, sj) or not _is_free_cell(graph, gi, gj):
        return []

    queue = deque()

    # Initialize start cell.
    graph.visited[sj, si] = True
    graph.parent[sj, si] = [-1, -1]  # start has no parent
    queue.append((si, sj))
    graph.visited_cells.append(Cell(si, sj))

    while queue:
        i, j = queue.popleft()

        # If we reached the goal, build and return the path.
        if i == gi and j == gj:
            return trace_path(Cell(gi, gj), graph)

        # Explore neighbors.
        for ni, nj in graph.find_neighbors(i, j):
            if not _is_free_cell(graph, ni, nj):
                continue
            if graph.visited[nj, ni]:
                continue

            graph.visited[nj, ni] = True
            graph.parent[nj, ni] = [i, j]
            queue.append((ni, nj))
            graph.visited_cells.append(Cell(ni, nj))

    # No path found.
    return []

