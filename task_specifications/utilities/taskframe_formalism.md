# Task specification using the task-frame formalism in eTaSL

The file `taskframe-formalism.lua` contains a library for
using the taskframe formalism of [^2] within eTaSL [^1], possibly
combining this with more advanced constraints, and possibly using
multiple taskframes.

> [!NOTE]
> **TODO:**
> - For the moment, tracking on velocities is not implemented.
> - For the moment, gravity compensation is not tested.
> - There is no (automatic) calibration procedure for the mass parameters for gravity compensation.
> - There is no (automatic) calibration procedure for the stiffness.

The `taskframe-formalism.lua` is a small library to translate the task frame
formalism of [^2] within eTaSL.

We first start by defining the task frame, using an expression for its pose,
for the associated force measurement `meas`, the estimated environmental
stiffness `Ke`, the force control constant `K` and the tracking control
constant for the orientation `K_adapt`. The force sensor frame is described
by `ft_frame`. In order to compensate for the weight of the tool and
compute the externally applied forces, the center of gravity of the tool (after
the force sensor) is given by `ft_cog`, its mass by `ft_mass`. We also
specify the gravity vector using `gravity`. We do not take into account
inertial effects.

The measured forces and the desired forces specified are the forces acting
**on** the robot (so, the negative of the forces that the robot exerts on the
environment). In other words, we make a free body diagram of the **robot**,
and follow standard conventions in mechanical engineering.

```lua
TF = TaskFrame:new {
    context    = ctx,
    taskframe  = ee*R,
    K          = 4,
    K_adapt    = 0.9,
    Ke         = 5000,
    meas       = wrench(vector(Fx,Fy,Fz),vector(Tx,Ty,Tz)),
    ft_frame   = ee,
    ft_cog     = origin(ee),
    ft_mass    = 0.0,
    gravity    = Vector(0,0,-9.81),
    priority   = 1
}
```

The above generates a function `move_compliantly` that can be
used to specify tasks within the task frame formalism:

```lua
TF:setForce(                X,  30            )
TF:setVelocity(             Y,  0.02          )
TF:setVelocity(             Z,  0             )
TF:setRotationalVelocity(   X,  0             )
TF:setRotationalVelocity(   Y,  0             )
TF:trackOrientationOnForce( Z,  Vector(1,0,0) )
```

The following methods are available:

```lua
setVelocity : sets the velocity in a direction.
setRotationalVelocity : set the desired rotational velocity in a direction.
setForce(dir, force) : sets the desired `force` in a direction.
setTorque(dir, torque) : sets the desired `torque` in a direction.
trackOrientationOnForce(dir, alignTo)    :
    adapt the orientation of the taskframe by rotating in direction `dir`
    such that the resulting external force is oriented along `alignTo`.
```

Using a lua table as last argument you can give additional parameters overriding
the defaults coming from the `TaskFrame:new()`.

The following methods get some information back for further use:

```lua
createPathlength,
magnitudeVelocity,
externalWrench,
```

There is also a routine to compute the path length.

```lua
PL = TF:createPathlength()
```

We can use this path length to construct a stop-condition
for our task.

If you do not want to let the end effector move together with
the task frame (e.g. when using a roller), you can use a
feature variable. If you do not want the end effector to move,
you will need an additional constraint to specify this.
The contour following example uses this approach.

[^1]: Aertbeliën, E., & De Schutter, J. (2014, September). eTaSL/eTC: A
    constraint-based task specification language and robot controller using
    expression graphs. In 2014 IEEE/RSJ International Conference on Intelligent
    Robots and Systems (pp. 1540-1546). IEEE.

[^2]: Bruyninckx, H., & De Schutter, J. (1996). Specification of
    force-controlled actions in the "task frame formalism"-a synthesis. IEEE
    transactions on robotics and automation, 12(4), 581-589.
