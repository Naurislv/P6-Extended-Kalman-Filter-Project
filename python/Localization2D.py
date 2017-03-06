"""Localization for 2D world. More info can be found in Localization (1D world implementation).

The function localize takes the following arguments:

colors:
    2D list, each entry either 'R' (for red cell) or 'G' (for green cell)

measurements:
    list of measurements taken by the robot, each entry either 'R' or 'G'

motions:
    list of actions taken by the robot, each entry of the form [dy,dx],
    where dx refers to the change in the x-direction (positive meaning
    movement to the right) and dy refers to the change in the y-direction
    (positive meaning movement downward)
    NOTE: the *first* coordinate is change in y; the *second* coordinate is change in x

sensor_right:
    float between 0 and 1, giving the probability that any given
    measurement is correct; the probability that the measurement is
    incorrect is 1-sensor_right

p_move:
    float between 0 and 1, giving the probability that any given movement
    command takes place; the probability that the movement command fails
    (and the robot remains still) is 1-p_move; the robot will NOT overshoot
    its destination in this exercise

The function should RETURN (not just show or print) a 2D list (of the same
dimensions as colors) that gives the probabilities that the robot occupies
each cell in the world.

Compute the probabilities by assuming the robot initially has a uniform
probability of being in any cell.

Also assume that at each step, the robot:
    1) first makes a movement,
    2) then takes a measurement.

Motion:
    [0,0] - stay
    [0,1] - right
    [0,-1] - left
    [1,0] - down
    [-1,0] - up
"""

colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R']
motions = [[0, 0]]
sensor_right = 0.8
p_move = 1.0


def localize(colors, measurements, motions, sensor_right, p_move):
    """Localize robot in 2D world."""
    pHit = sensor_right
    pMiss = 1 - sensor_right

    # initialize p to a uniform distribution over a grid of the same dimensions as colors.
    # pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    # p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

    for measurement in measurements:
        good_count = sum([row.count(measurement) for row in colors])
        total_count = len(colors[0]) * len(colors)
        good_weight = pHit * good_count
        false_weight = pMiss * (total_count - good_count)
        weight_sum = good_weight + false_weight

        good_weight = good_weight / weight_sum / good_count
        false_weight = false_weight / weight_sum / (total_count - good_count)

        p = []
        for row in range(len(colors)):
            p.append([])
            for col in range(len(colors[0])):
                if colors[row][col] == measurement:
                    p[row].append(good_weight)
                else:
                    p[row].append(false_weight)

    return p

print(localize(colors, measurements, motions, sensor_right, p_move))
