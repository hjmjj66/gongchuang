"""Screen road numbers are independent of the planner's EDGES tuple order."""

ROUTE_EDGES = {
    1: (7, 4), 2: (4, 1), 3: (7, 8), 4: (4, 5),
    5: (1, 2), 6: (8, 5), 7: (5, 2), 8: (8, 9),
    9: (5, 6), 10: (2, 3), 11: (9, 6), 12: (6, 3),
}


def parse_screen_config(start_zone, stop_zone, forbidden_routes):
    if start_zone not in (1, 2) or stop_zone not in (1, 2):
        raise ValueError('start/stop 必须分别为 1 或 2')
    if any(number not in ROUTE_EDGES for number in forbidden_routes):
        raise ValueError('禁行路径编号必须为 1～12')
    routes = tuple(sorted(set(forbidden_routes)))
    edges = tuple(sorted(tuple(sorted(ROUTE_EDGES[number])) for number in routes))
    return (1 if start_zone == 1 else 3), (1 if stop_zone == 1 else 3), routes, edges
