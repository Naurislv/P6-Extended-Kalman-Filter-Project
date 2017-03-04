"""Localization lesson from AI for Robotics. Free course.

https://classroom.udacity.com/courses/cs373/lessons/48739381/concepts/487214690923

Provide all possible state vector name world. Then measurement is any Robot (sensor)
state measurements. The goal is to compute new probability vector q from Distribution
p and then make a movement or shift probability distribution. This can be done by
manually choosing pHit and pMiss. Where pHit means when measurement is equel to state
in world and pMiss means when measurement is not equal to state in world. Then we just
multiply Distribution by pHit or pMiss values when state is equal to measurement Z.
Then using function move we make a movement, but this movement is not perfect - Robot
does not always move where it inteded, so we give probability for it's movements pExact,
pOvershoot, pUndershoot.

measurements: list of measurements
World: all possible states
p: probability distribution of all possible states
pHit: multiplier when measurements[i] == World[i] state
pMiss: multiplier when measurements[i != World[i] state

motions: list of motions robot takes. First we take measurement[i] then we take motion[i]
         then measurement[i+1] and motion[i+1] ...

pExact: probability that robot will move where it intended to
pOvershoot: probability that robot will move one step further
pUndershoot: probability that robot will move one step back
"""

p = [0.2, 0.2, 0.2, 0.2, 0.2]
world = ['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
motions = [1, 1]
pHit = 0.6
pMiss = 0.2

pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


def sense(p, Z):
    """Multiply distribution vector by state multiplyer.

    Z = single measurement
    """
    q = []
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1 - hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q


def move(p, U):
    """Move U positions to direction. p: distribution.

    Robot moves as accurate as p_of_movement told to.

    example of p_of_movement :
        p_of_movement = [0.1, 0.8, 0.1]
        This means that it has 0.8 probability to move in right positions and 0.1
        probability to move in neighbor positions.

    p_of_movement we now apply to our distribution vector.
    """
    q = []
    p_len = len(p)
    for i in range(len(p)):
        s = pExact * p[(i - U) % p_len]  # target step
        s += pOvershoot * p[(i - U - 1) % p_len]  # left neighbor step
        s += pOvershoot * p[(i - U - 1) % p_len]  # right neighbor step
        q.append(s)

    return q


def entropy():
    """Measure of information that distribution has."""

    # not implemented yet

# compute p after multiple motions.
for i in range(len(motions)):
    p = sense(p, measurements[i])
    p = move(p, motions[i])

print(p)
