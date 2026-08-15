# Pathing
## Algorithms

### BFS


Pseudocode:
```
BFS(grid, start, goal):
    if start is an obstacle:
        start = first reachable free cell near start
    mark start as visited
    enqueue start to end of queue
    while queue is not empty:
        current = dequeue front
        for each of the 8 neighboring cells:
            if neighbor is inside grid and neighbor is not visited and neighbor is not an obstacle:
                mark neighbor as visited
                neighbor.parent = current
                enqueue neighbor to end of queue 
                if neighbor is goal:
                    return path reconstructed through parent pointers
    return no path
```

Notes:
- We use a secondary BFS to find the first reachable free cell near the start.

## Spline
### Knot Vector
$$
u_i =
\begin{cases}
0, & 0 \le i \le p, \\[4pt]
\dfrac{i-p}{n-p}, & p < i < n \\[4pt]
1, & n \le i \le n+p.
\end{cases}
$$
Where:
- $u_i$ is the knot at index $i$.
- $i$ is the knot index.
- $n$ is the number of control points.
- $p$ is the spline degree.
The knot vector divides the spline parameter into polynomial segments. It determines which control points affect each segment and the smoothness at segment boundaries. For a cubic spline, the first and last four knots are repeated, clamping the spline to its endpoint control points, while uniformly spaced interior knots produce smooth transitions.
### Basis Function
$$
C(t)=\sum_{i=0}^{n-1}N_{i,p}(t)P_i
$$
Where:
- $C(t)$ is the point on the spline at parameter $t$
- $t$ is the position along the spline parameter range
- $n$ is the number of control points
- $i$ is the control point index
- $N_{i,p}(t)$ is the basis value for control point $i$ at $t$
- $p$ is the spline degree
- $P_i$ is control point $i$
	- this is coming from bfs
The basis functions determine how strongly each control point influences the spline at a given parameter value. Each basis value is multiplied by its corresponding control point, and the results are added to get the final point on the spline.

### Finite differences
$$
D_i^{(0)}=P_i
$$
$$
D_i^{(k)}=
\frac{D_{i+1}^{(k-1)}-D_i^{(k-1)}}
{u_{i+p+1}-u_{i+k}}
$$
Where:
- $D_i^{(k)}$ is the finite difference of order $k$
- $P_i$ is control piont at index $i$
- $u_i$ is knot at index $i$
- $p$ is the spline degree
- $k$ is the derivative order
Finite differences are used to find how the control points change relative to the knot parameter. We take the difference between two neighboring control points and divide it by the distance between their knots. This creates new control points that describe the spline’s direction and rate of change, rather than its position. The parameter isn't physical time, but it can later be used to calculate the tangent, velocity, or acceleration.

### Evaluate positioin
$$
\operatorname{EvaluatePosition}(t,\ controls,\ knots,\ p)
=
\sum_{i=0}^{n-1}
\operatorname{Basis}(i,\ p,\ t,\ knots)\cdot controls_i
$$
Evaluates the spline's $x$ and $y$ at given parameter $t$
### Evaluate derivative
$$
\operatorname{EvaluateDerivative}(t,\ controls,\ knots,\ p,\ k)
=
\frac{p!}{(p-k)!}
\operatorname{EvaluatePosition}
\left(
t,\ 
\operatorname{FiniteDifferences}(controls,\ knots,\ p,\ k),\
knots,\
p-k
\right)
$$
Where:
- $k$ is the derivative order
- $t$ is position along the spline parameter range
- $p$ is the spline degree
- $knots$ is the knot vector from the function above
This evaluates the spline’s derivative with respect to its parameter, giving the tangent and relative rate of change along the path. We use this as a direction and not a raw velocity, so we can feed that direction along with robot max accel and max velo (constants were stolen from team pathplanner config) to the velocity profile which will generate our velocities.


## Path following

1. Find the closest velocity profile point to the current position of the robot
2. Take that feedforward velocity from the profile point and drive along it
3. A P controller (from PID) keeps the robot more on track so it doesn't drift too far off the path
4. If the robot is too far off the path, we replan to make sure we don't get permanently offset and crash
5. Stop when the robot is within the target radius
	- if this isn't accurate enough, we can also linearly approach the target point when we get into a circle around it
