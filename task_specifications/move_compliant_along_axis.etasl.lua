--  Copyright (c) 2025 KU Leuven, Belgium
--
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
require("math")
tff=require("taskframe_formalism")
reqs = require("task_requirements")

task_description = [[Compliant motion along one axis: the robot pushes or pulls along one axis (parameter axis) of the control frame (task_frame*tf_2_insframe) with a desired force (parameter force), while it is compliant in all other directions: the desired force is 0 along the other two axes and the desired torque is 0 around all three axes. It can be used to approach an object until contact (the first contact stops it), and to pull a handle of a drawer or a door: a drawer moves in a straight line, a door follows a circular arc, because the robot gives way to the forces and torques that the handle exerts on its grasp.
The control is formulated in the instantaneous control frame (the frame that coincides with the control frame at every instant). Along each axis the velocity is K_force*(desired force - measured force)/K_F, around each axis the angular velocity is K_torque*(desired torque - measured torque)/K_T. The measured wrench is the wrench that the end effector exerts on the environment (convention of the MuJoCo simulator of the Crospi Playground and of mujoco_robotsimulator_crospi: not the convention of guarded_motion), read from the input channel wrench_input_channel (type wrench, expressed in FT_sensor_frame with its origin as reference point). It is passed through a dead zone and expressed in the control frame, with its origin as reference point. There is no gravity compensation: the sensor of the simulator measures only contact forces.
The distance travelled along the axis is the integral of the velocity of the control frame along its (instantaneous) axis, computed with a feature variable and a constraint (an integrator): it is the length of the path (for a door a circular arc: the path length along the axis that rotates with the handle) and not the distance to the start pose. The task finishes (monitor action "exit") when the absolute value of this distance reaches the parameter distance. It also finishes when the contact monitor is active (contact_force > 0) and the force exerted in the direction of the motion exceeds contact_force (after time_delay), and when max_time (if > 0) has elapsed. Outputs: travelled distance (travel), pose of the control frame (x_tf, y_tf, z_tf, roll_tf, pitch_tf, yaw_tf, task_frame_c) and the processed wrench in the control frame (x_force ... z_torque).]]

-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.enum({name="axis", type=reqs.enum_types.string, default="z", description="Axis of the control frame (task_frame*tf_2_insframe) along which the robot pushes or pulls. The direction is given by the sign of force. The distance travelled is measured along it", required=true, accepted_vals = {"x","y","z"}}),
    reqs.params.scalar({name="force", description="Desired force [N] exerted by the robot on the environment along axis, in the control frame. Its sign is the direction of the motion: the robot moves along +axis if it is positive (it pushes in the direction +axis), along -axis if it is negative (it pulls). In free space the velocity along the axis is K_force*force/K_F (limited by maxvel); in contact the robot stops moving when the measured force reaches this value, so a force larger than the resisting force (friction of a drawer or door) is needed to move it", default = 5.0, required=true}),
    reqs.params.scalar({name="distance", description="Distance [m] travelled along axis (integral of the velocity along the axis of the control frame, see the description of the task) after which the task finishes. Its sign is ignored. It acts as a maximum when contact_force is used", default = 0.1, required=true, minimum = 0.0}),
    reqs.params.scalar({name="contact_force", description="Contact threshold [N] for a guarded motion: the task finishes when the force exerted by the robot along axis in the direction of the motion exceeds this value (after time_delay). 0 disables this monitor (the task then only finishes by distance or max_time). It should be smaller than the absolute value of force, otherwise the robot stops before reaching it", default = 0.0, required=false, minimum = 0.0}),
    reqs.params.scalar({name="time_delay", description="Time [s] after the start of the task during which the contact monitor (contact_force) is disabled, to ignore forces present at the beginning", default = 0.0, required=false, minimum = 0.0}),
    reqs.params.scalar({name="max_time", description="Maximum duration [s] of the task: it finishes when this time has elapsed. 0 disables this monitor", default = 0.0, required=false, minimum = 0.0}),
    reqs.params.scalar({name="maxvel", description="Maximum absolute velocity [m/s] of the control frame along each of its axes", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="max_angular_velocity", description="Maximum absolute angular velocity [rad/s] of the control frame around each of its axes", default = 0.5, required=false, minimum = 0.0}),
    reqs.params.array({name="K_F", type=reqs.array_types.number, default={400, 400, 1200},
                            description="Diagonal elements [N/m] of the translational stiffness, as [x,y,z] along the axes of the control frame, used as model of the force control: the velocity along each axis is K_force*(desired force - measured force)/K_F. Lower values make the robot faster and more compliant along that axis, higher values slower and stiffer. The compliant axes should have a low value so that the robot follows the object (e.g. the arc of a door) with a small force", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.array({name="K_T", type=reqs.array_types.number, default={10, 10, 10},
                            description="Diagonal elements [Nm/rad] of the rotational stiffness, as [x,y,z] around the axes of the control frame, used as model of the torque control: the angular velocity around each axis is K_torque*(0 - measured torque)/K_T, i.e. the robot rotates with the object (e.g. a door handle that turns) with a small torque. Lower values make the robot rotate faster for the same torque", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="K_force", description="Feedback gain [1/s] of the force control constraints along x, y and z (inverse of their time constant)", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="K_torque", description="Feedback gain [1/s] of the torque control constraints around x, y and z (inverse of their time constant)", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_force", description="Weight [-] of the force control constraints relative to the other constraints of the same priority. Only matters when the constraints conflict", default = 1, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_torque", description="Weight [-] of the torque control constraints relative to the other constraints of the same priority. Only matters when the constraints conflict", default = 1, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_max_velocity", description="Weight [-] of the constraints that limit the velocities (higher priority than the force and torque control)", default = 10, required=false, minimum = 0.0}),
    reqs.params.scalar({name="force_threshold", description="Dead zone [N] applied to each force component of the measured wrench (in FT_sensor_frame): components with an absolute value below it are set to zero and larger ones are reduced by it. 0 disables the dead zone", default = 0.5, required=false, minimum = 0.0}),
    reqs.params.scalar({name="torque_threshold", description="Dead zone [Nm] applied to each torque component of the measured wrench (in FT_sensor_frame). 0 disables the dead zone", default = 0.05, required=false, minimum = 0.0}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (from the robot model) that is controlled, e.g. the tool center point. The control frame is defined w.r.t. this frame by tf_2_insframe", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of the robot frame (from the robot model) of the force/torque sensor. The measured wrench of the input channel is expressed in this frame, with its origin as reference point", default = "FT_sensor_frame", required=true}),
    reqs.params.array({name="tf_2_insframe", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                            description="Pose of the control frame w.r.t. task_frame, as [x,y,z,qx,qy,qz,qw] (position in m, unit quaternion with the real part qw last), expressed in task_frame. The axis of the motion, the forces and the torques are expressed in this frame, and the torques are w.r.t. its origin (e.g. the center of the handle that is grasped). Use it when the gripper is not aligned with the direction of the motion (e.g. a tool tilted downwards that pulls horizontally). The default is the identity", required=true, minItems = 7, maxItems = 7}),
    reqs.params.string({name="wrench_input_channel", description="Name of the input channel (type wrench) from which the measured wrench of the force/torque sensor is read: the channel fed directly by the robot driver/simulator (name_expr_cartesian_wrench of the MuJoCo simulator) or the varname of an input handler that provides it. The wrench is the wrench exerted by the end effector on the environment", default = "wrench_input", required=false}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"),
    param.get("FT_sensor_frame"),
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    return conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
end

-- ========================================= PARAMETERS ===================================
axis             = param.get("axis")
desired_force    = param.get("force")
distance         = param.get("distance")
contact_force    = param.get("contact_force")
time_delay       = constant(param.get("time_delay"))
max_time         = param.get("max_time")
maxvel           = param.get("maxvel")
max_angular_vel  = param.get("max_angular_velocity")
K_F              = param.get("K_F")
K_T              = param.get("K_T")
force_threshold  = constant(param.get("force_threshold"))
torque_threshold = constant(param.get("torque_threshold"))
tf_2_ins         = param.get("tf_2_insframe")

direction = 1.0
if desired_force < 0 then direction = -1.0 end

-- control frame w.r.t. task_frame
quat_tf_2_insframe = quaternion(constant(tf_2_ins[7]), vector(constant(tf_2_ins[4]), constant(tf_2_ins[5]), constant(tf_2_ins[6])))
tf_2_insframe = frame(toRot(quat_tf_2_insframe), vector(constant(tf_2_ins[1]), constant(tf_2_ins[2]), constant(tf_2_ins[3])))
task_frame_c  = task_frame*tf_2_insframe

-- ========================================= Variables coming from topic input handlers ===================================
sensed_wrench = ctx:createInputChannelWrench(param.get("wrench_input_channel"))

-- =============================== MEASURED WRENCH IN THE CONTROL FRAME ==============================
wrench_dead_zone = wrench(
    vector(dead_zone(coord_x(force(sensed_wrench)), force_threshold),
           dead_zone(coord_y(force(sensed_wrench)), force_threshold),
           dead_zone(coord_z(force(sensed_wrench)), force_threshold)),
    vector(dead_zone(coord_x(torque(sensed_wrench)), torque_threshold),
           dead_zone(coord_y(torque(sensed_wrench)), torque_threshold),
           dead_zone(coord_z(torque(sensed_wrench)), torque_threshold)))

wrench_cf = ref_point(transform(rotation(inv(task_frame_c)*FT_sensor_frame), wrench_dead_zone),
                      -origin(inv(task_frame_c)*FT_sensor_frame))

Fx = coord_x(force(wrench_cf))
Fy = coord_y(force(wrench_cf))
Fz = coord_z(force(wrench_cf))
Tx = coord_x(torque(wrench_cf))
Ty = coord_y(torque(wrench_cf))
Tz = coord_z(torque(wrench_cf))

-- =============================== INSTANTANEOUS FRAME ==============================
-- pose of the control frame w.r.t. its pose at the current instant (zero, but its derivative is the velocity of the
-- control frame expressed in the control frame)
tf_inst = inv(make_constant(task_frame_c))*task_frame_c
pos_inst = origin(tf_inst)
rot_inst = getRotVec(rotation(tf_inst))

-- =============================== FORCE AND TORQUE CONTROL ==============================
-- velocity along each axis: K_force*(desired force - measured force)/K_F  (measured force: exerted on the environment)
local axes     = {"x", "y", "z"}
local coord_of = {x = coord_x, y = coord_y, z = coord_z}
local measured_force  = {x = Fx, y = Fy, z = Fz}
local measured_torque = {x = Tx, y = Ty, z = Tz}

for i, a in ipairs(axes) do
    local desired = 0.0
    if a == axis then desired = desired_force end
    Constraint{
        context  = ctx,
        name     = "force_" .. a,
        model    = constant(K_F[i]) * coord_of[a](pos_inst),
        meas     = measured_force[a],
        target   = constant(desired),
        K        = constant(param.get("K_force")),
        priority = 2,
        weight   = constant(param.get("weight_force")),
    }
    Constraint{
        context  = ctx,
        name     = "torque_" .. a,
        model    = constant(K_T[i]) * coord_of[a](rot_inst),
        meas     = measured_torque[a],
        target   = constant(0.0),
        K        = constant(param.get("K_torque")),
        priority = 2,
        weight   = constant(param.get("weight_torque")),
    }
    -- velocity limits (higher priority)
    Constraint{
        context      = ctx,
        name         = "max_velocity_" .. a,
        expr         = coord_of[a](pos_inst),
        target_upper = maxvel*time,
        target_lower = -maxvel*time,
        K            = 0,
        weight       = param.get("weight_max_velocity"),
        priority     = 1
    }
    Constraint{
        context      = ctx,
        name         = "max_angular_velocity_" .. a,
        expr         = coord_of[a](rot_inst),
        target_upper = max_angular_vel*time,
        target_lower = -max_angular_vel*time,
        K            = 0,
        weight       = param.get("weight_max_velocity"),
        priority     = 1
    }
end

-- =============================== DISTANCE TRAVELLED (INTEGRATOR) ==============================
-- The feature variable "travel" is the integral of the velocity of the control frame along its axis: the constraint
-- (K = 0) imposes that its derivative equals the derivative of the position along the axis in the instantaneous frame.
travel = Variable{
    context = ctx,
    name    = "travel",
    vartype = "feature",
    initial = 0.0,
}

Constraint{
    context  = ctx,
    name     = "distance_integrator",
    expr     = travel - coord_of[axis](pos_inst),
    K        = 0,
    priority = 1,
    weight   = constant(1),
}

-- =============================== MONITORS ==============================
Monitor{
    context    = ctx,
    name       = "finish_after_distance",
    upper      = distance,
    actionname = "exit",
    expr       = abs(travel),
}

if contact_force > 0 then
    -- force exerted in the direction of the motion, enabled after time_delay
    Monitor{
        context    = ctx,
        name       = "finish_on_contact",
        upper      = 1.0,
        actionname = "exit",
        expr       = conditional(time - time_delay, 1, 0) * (direction * measured_force[axis]) / constant(contact_force),
    }
end

if max_time > 0 then
    Monitor{
        context    = ctx,
        name       = "finish_after_max_time",
        upper      = max_time,
        actionname = "exit",
        expr       = time,
    }
end

-- =============================== OUTPUT ==============================
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame_c))

ctx:setOutputExpression("travel"       , travel)
ctx:setOutputExpression("x_tf"         , coord_x(origin(task_frame_c)))
ctx:setOutputExpression("y_tf"         , coord_y(origin(task_frame_c)))
ctx:setOutputExpression("z_tf"         , coord_z(origin(task_frame_c)))
ctx:setOutputExpression("roll_tf"      , roll_tf)
ctx:setOutputExpression("pitch_tf"     , pitch_tf)
ctx:setOutputExpression("yaw_tf"       , yaw_tf)
ctx:setOutputExpression("x_force"      , Fx)
ctx:setOutputExpression("y_force"      , Fy)
ctx:setOutputExpression("z_force"      , Fz)
ctx:setOutputExpression("x_torque"     , Tx)
ctx:setOutputExpression("y_torque"     , Ty)
ctx:setOutputExpression("z_torque"     , Tz)
ctx:setOutputExpression("task_frame_c" , task_frame_c)
