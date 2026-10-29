"""Small Dijkstra search: minimize (distance, translation direction changes).

The search state includes the previous movement direction. Keeping only the best
cost per node can discard a route that produces fewer changes on the next edge.
Vehicle heading is constant during translations, independent of these changes.
"""

from dataclasses import dataclass
from enum import IntEnum
import heapq
from typing import Iterable, Mapping, Optional, Tuple


HEADINGS = ('UP', 'RIGHT', 'DOWN', 'LEFT')
# Map columns from left to right and rows from top to bottom.
POSITIONS = {node: (2 - (node - 1) // 3, (node - 1) % 3)
             for node in range(1, 10)}
EDGES = (
    (7, 4), (4, 1), (8, 5), (5, 2), (9, 6), (6, 3),
    (7, 8), (8, 9), (4, 5), (5, 6), (1, 2), (2, 3),
)


class Opcode(IntEnum):
    FORWARD_ONE_EDGE = 1
    TURN_LEFT_90 = 2
    TURN_RIGHT_90 = 3
    STRAFE_LEFT_ONE_EDGE = 4
    STRAFE_RIGHT_ONE_EDGE = 5
    BACKWARD_ONE_EDGE = 6
    DIAGONAL_FORWARD_LEFT = 7
    DIAGONAL_FORWARD_RIGHT = 8
    DIAGONAL_BACKWARD_LEFT = 9
    DIAGONAL_BACKWARD_RIGHT = 10


@dataclass(frozen=True)
class PathResult:
    nodes: Tuple[int, ...]
    total_distance_mm: int
    direction_changes: int


@dataclass(frozen=True)
class Step:
    opcode: Opcode
    from_node: int
    to_node: int
    distance_mm: int
    target_heading: str


def parking_step(node: int, heading: str, maneuver: str, offset_mm: int) -> Step:
    # Map y points down; enter node 1 towards upper-right, node 3 towards lower-right.
    dx, dy = 1, -1 if node == 1 else 1
    if maneuver == 'EXIT':
        dx, dy = -dx, -dy
    fx, fy = ((0, -1), (1, 0), (0, 1), (-1, 0))[HEADINGS.index(heading)]
    forward, left = dx * fx + dy * fy, dx * fy - dy * fx
    opcode = {
        (1, 1): Opcode.DIAGONAL_FORWARD_LEFT,
        (1, -1): Opcode.DIAGONAL_FORWARD_RIGHT,
        (-1, 1): Opcode.DIAGONAL_BACKWARD_LEFT,
        (-1, -1): Opcode.DIAGONAL_BACKWARD_RIGHT,
    }[forward, left]
    return Step(opcode, node, node, offset_mm, heading)


def check_node(node: int) -> int:
    if type(node) is not int or node not in POSITIONS:
        raise ValueError(f'node must be an integer in 1..9, got {node!r}')
    return node


def normalize_heading(value: str) -> str:
    if not isinstance(value, str) or value.strip().upper() not in HEADINGS:
        raise ValueError(f'heading must be UP/RIGHT/DOWN/LEFT, got {value!r}')
    return value.strip().upper()


def edge_key(a: int, b: int) -> Tuple[int, int]:
    check_node(a)
    check_node(b)
    return min(a, b), max(a, b)


def movement_direction(a: int, b: int) -> int:
    ax, ay = POSITIONS[check_node(a)]
    bx, by = POSITIONS[check_node(b)]
    offsets = ((0, -1), (1, 0), (0, 1), (-1, 0))
    try:
        return offsets.index((bx - ax, by - ay))
    except ValueError:
        raise ValueError(f'{a}->{b} is not an adjacent map edge') from None


class Topology:
    def __init__(self, default_edge_length_mm: int = 1200,
                 edge_lengths: Optional[Mapping[Tuple[int, int], int]] = None):
        self._check_length(default_edge_length_mm)
        self.lengths = {edge_key(a, b): default_edge_length_mm for a, b in EDGES}
        for (a, b), length in (edge_lengths or {}).items():
            key = self.check_edge(a, b)
            self._check_length(length)
            self.lengths[key] = length
        self.neighbors = {node: [] for node in POSITIONS}
        for a, b in self.lengths:
            self.neighbors[a].append(b)
            self.neighbors[b].append(a)
        for neighbors in self.neighbors.values():
            neighbors.sort()

    @staticmethod
    def _check_length(length: int):
        if type(length) is not int or not 0 < length <= 4294967295:
            raise ValueError('edge length must be an integer in 1..4294967295 mm')

    def check_edge(self, a: int, b: int) -> Tuple[int, int]:
        key = edge_key(a, b)
        if key not in self.lengths:
            raise ValueError(f'({a}, {b}) is not one of the 12 map edges')
        return key

    def normalize_blocked(self, blocked: Iterable[Tuple[int, int]]):
        return frozenset(self.check_edge(a, b) for a, b in blocked)

    def shortest_path(self, start: int, goal: int,
                      blocked: Iterable[Tuple[int, int]] = ()) -> Optional[PathResult]:
        check_node(start)
        check_node(goal)
        closed = self.normalize_blocked(blocked)
        # Lexicographic node sequence breaks any remaining ties reproducibly.
        initial = (0, 0, (start,))
        best = {(start, -1): initial}
        queue = [(*initial, -1)]
        while queue:
            distance, changes, path, previous = heapq.heappop(queue)
            node = path[-1]
            if best.get((node, previous)) != (distance, changes, path):
                continue
            if node == goal:
                return PathResult(path, distance, changes)
            for neighbor in self.neighbors[node]:
                key = edge_key(node, neighbor)
                if key in closed:
                    continue
                direction = movement_direction(node, neighbor)
                candidate = (
                    distance + self.lengths[key],
                    changes + int(previous != -1 and direction != previous),
                    path + (neighbor,),
                )
                state = (neighbor, direction)
                if state not in best or candidate < best[state]:
                    best[state] = candidate
                    heapq.heappush(queue, (*candidate, direction))
        return None

    def build_steps(self, nodes: Iterable[int], current_heading: str,
                    goal_heading: Optional[str] = None) -> Tuple[Step, ...]:
        heading = normalize_heading(current_heading)
        final_heading = normalize_heading(goal_heading) if goal_heading is not None else None
        path = tuple(nodes)
        if not path:
            raise ValueError('cannot build actions for an empty path')
        for node in path:
            check_node(node)
        relative_opcodes = (
            Opcode.FORWARD_ONE_EDGE, Opcode.STRAFE_RIGHT_ONE_EDGE,
            Opcode.BACKWARD_ONE_EDGE, Opcode.STRAFE_LEFT_ONE_EDGE,
        )
        steps = []
        for a, b in zip(path, path[1:]):
            key = self.check_edge(a, b)
            relative = (movement_direction(a, b) - HEADINGS.index(heading)) % 4
            steps.append(Step(relative_opcodes[relative], a, b, self.lengths[key], heading))
        if final_heading is not None:
            delta = (HEADINGS.index(final_heading) - HEADINGS.index(heading)) % 4
            # 180 degrees always uses two right rotations for deterministic output.
            rotations = ([Opcode.TURN_LEFT_90] if delta == 3 else
                         [Opcode.TURN_RIGHT_90] * delta)
            for opcode in rotations:
                increment = -1 if opcode == Opcode.TURN_LEFT_90 else 1
                heading = HEADINGS[(HEADINGS.index(heading) + increment) % 4]
                steps.append(Step(opcode, path[-1], path[-1], 0, heading))
        return tuple(steps)
