"""Measured odometry display calibration. Never feeds navigation."""

import math

# Pixel positions on the original 1035x916 screenshot (road center anchors).
ANCHORS = {1: (821, 141), 2: (821, 482), 3: (821, 800),
           4: (500, 141), 5: (500, 482), 6: (500, 800),
           7: (180, 141), 8: (180, 482), 9: (180, 800)}
HEADINGS = {'UP': 0.0, 'RIGHT': 90.0, 'DOWN': 180.0, 'LEFT': 270.0}


def pose_at(node, heading):
    x, y = ANCHORS[node]
    return dict(x=x, y=y, angle=HEADINGS[heading], valid=True, edge=None)


def measured_pose(odom, anchor, node, heading, lengths):
    """Measured 2D displacement; never snap to route or endpoint.

    Mean parallel road lengths define a display grid. Inconsistent rectangular
    calibration is flagged. Display calibration never feeds the controller.
    """
    columns = ((7, 8, 9), (4, 5, 6), (1, 2, 3))
    rows = ((7, 4, 1), (8, 5, 2), (9, 6, 3))

    def interval(groups, index):
        values = [lengths.get(tuple(sorted(pair)), 1200.)/1000.
                  for pair in zip(groups[index], groups[index+1])]
        return sum(values)/len(values), max(values)-min(values) > .001

    w0, warn0 = interval(columns, 0)
    w1, warn1 = interval(columns, 1)
    h0, warn2 = interval(rows, 0)
    h1, warn3 = interval(rows, 1)
    xs, ys = (0., w0, w0+w1), (0., h0, h0+h1)
    col = next(i for i, group in enumerate(columns) if node in group)
    row = next(i for i, group in enumerate(rows) if node in group)
    dx, dy = odom['x_m']-anchor['x_m'], odom['y_m']-anchor['y_m']
    theta = anchor['yaw_rad']
    forward = math.cos(theta)*dx+math.sin(theta)*dy
    left = -math.sin(theta)*dx+math.cos(theta)*dy
    h = math.radians(HEADINGS[heading])
    mx = xs[col]-math.cos(h)*left+math.sin(h)*forward
    my = ys[row]-math.sin(h)*left-math.cos(h)*forward

    def pixels(value, metric, image):
        i = 0 if value <= metric[1] else 1
        return image[i]+(value-metric[i])/(metric[i+1]-metric[i])*(image[i+1]-image[i])

    return dict(x=pixels(mx, xs, (180,500,821)), y=pixels(my, ys, (141,482,800)),
                angle=HEADINGS[heading]-(odom['yaw_deg']-anchor['yaw_deg']),
                valid=bool(odom['valid']), edge=None,
                calibration_warning=bool(warn0 or warn1 or warn2 or warn3))
