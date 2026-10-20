--  Copyright (c) 2026 KU Leuven, Belgium
--
--  Author: Erwin Aertbeliën and Santiago Iregui
--  emails: <erwin.aertbelien@kuleuven.be>, <santiago.iregui@kuleuven.be>
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

-- ==========================================================================================
-- 2D contour following with the task frame formalism [Bruyninckx and De Schutter, 1996].
--
-- The robot must ALREADY be in contact with the contour when this task starts (e.g. after a
-- guarded motion). The contour lies in a plane perpendicular to the tool axis, i.e. the
-- z-axis of the contact frame (see the parameter tf_2_contact_frame).
--
-- The task frame (tf) has its origin at the origin of the contact frame and is rotated around
-- its z-axis by an angle `alpha`, such that:
--   * x-axis: normal to the contour, along the contact force that the contour exerts ON the robot,
--   * y-axis: tangent to the contour,
--   * z-axis: tool axis, perpendicular to the plane of the contour.
-- In this task frame the task is specified as (task frame formalism):
--   * x  : force control, desired normal contact force `normal_force`,
--   * y  : velocity control, desired tangential velocity `tangential_velocity`,
--   * z  : keep the initial position along the tool axis,
--   * rot: keep the initial orientation of the tool,
--   * the task frame rotates around z (feature variable `alpha`) such that its x-axis stays
--     aligned with the measured contact force (i.e. it tracks the normal of the contour).
-- The robot itself does NOT rotate: only the task frame does (this is why `alpha` is a feature
-- variable). This suits rotationally symmetric tools (e.g. a cylinder or a roller).
-- The task finishes when the contact frame has travelled a distance `path_length`.
--
-- All forces used in the constraints are forces acting ON the robot by the environment (free
-- body diagram of the robot), as in the task frame formalism.
-- ==========================================================================================

require("context")
require("geometric")
require("velocities") -- for previous_velocity(..)
require("math")

reqs = require("task_requirements")

task_description = "This task specification implements 2D contour following with the task frame formalism: force control along the normal of the contour, velocity control along its tangent, and constant height and orientation of the tool. The task frame rotates around the tool axis to track the normal of the contour, estimated from the measured contact force. The robot must already be in contact with the contour when the task starts. The task finishes after travelling the specified path length."


-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques are measured", default = "FT_sensor_frame", required=true}),
    reqs.params.array({name="tf_2_contact_frame", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                            description="Transformation from the task_frame to the contact frame in [x,y,z,qx,qy,qz,qw]. The z-axis of the contact frame must be perpendicular to the plane of the contour (e.g. along the axis of a cylindrical tool)", required=true, minItems = 7, maxItems = 7}),
    reqs.params.enum({name="wrench_convention", type=reqs.enum_types.string, default="exerted_on_environment",
                            description="Sign convention of the measured wrench: wrench that the robot exerts on the environment (e.g. MuJoCo simulator) or wrench that the environment exerts on the robot", required=true, accepted_vals = {"exerted_on_environment","exerted_on_robot"}}),
    reqs.params.scalar({name="normal_force", description="Desired contact force [N] normal to the contour (pushing against the contour)", default = 10.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="tangential_velocity", description="Desired velocity [m/s] along the tangent of the contour. The sign selects the direction of motion along the contour", default = 0.02, required=true, minimum = -0.2, maximum = 0.2}),
    reqs.params.scalar({name="ramp_up_time", description="Time [s] to linearly increase the tangential velocity from zero to tangential_velocity, while the task frame aligns with the contour", default = 3.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="path_length", description="Distance [m] travelled along the contour after which the task finishes", default = 0.5, required=true, minimum = 0.0}),
    reqs.params.scalar({name="contact_stiffness", description="Estimated stiffness [N/m] of the contact (environment + robot) in the normal direction", default = 5000.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="K_force", description="Feedback gain [1/s] of the normal force control", default = 8.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="K_tracking", description="Feedback gain [1/s] of the rotation of the task frame that tracks the normal of the contour", default = 1.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="K_hold", description="Feedback gain [1/s] to keep the initial position along the tool axis and the initial orientation of the tool", default = 4.0, required=true, minimum = 0.0}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0},
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the force/torque sensor [N]", default = 0.0, required=true, minimum = 0.0}),
    reqs.params.scalar({name="filter_cutoff_frequency", description="Cut-off frequency [Hz] of the Butterworth low-pass filter applied to the measured wrench. Set to 0 to disable the filter. Keep it well below the Nyquist frequency of the controller (e.g. <= 10 Hz at 100 Hz)", default = 5.0, required=true, minimum = 0.0, maximum = 10.0}),
    reqs.params.int({name="filter_order", description="Order of the Butterworth low-pass filter (2, 3 or 4)", default = 2, required=true, minimum = 2, maximum = 4}),
})


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"),
    param.get("FT_sensor_frame"),
})
robot_joints    = robot.robot_joints
task_frame      = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))


-- ========================================= PARAMETERS ===================================
normal_force        = constant(param.get("normal_force"))
tangential_velocity = constant(param.get("tangential_velocity"))
ramp_up_time        = param.get("ramp_up_time")
path_length         = param.get("path_length")
contact_stiffness   = constant(param.get("contact_stiffness"))
K_force             = constant(param.get("K_force"))
K_tracking          = constant(param.get("K_tracking"))
K_hold              = constant(param.get("K_hold"))
tool_weight         = constant(param.get("tool_weight"))
tool_COG            = param.get("tool_COG")
tf_2_contact_frame  = param.get("tf_2_contact_frame")
filter_cutoff_frequency = param.get("filter_cutoff_frequency")
filter_order            = param.get("filter_order")

tool_COG = vector(constant(tool_COG[1]), constant(tool_COG[2]), constant(tool_COG[3]))

-- pose of the contact frame w.r.t. the task_frame (quaternion: real part last in the parameter)
quat_tf_2_contact  = quaternion(constant(tf_2_contact_frame[7]),
                                vector(constant(tf_2_contact_frame[4]), constant(tf_2_contact_frame[5]), constant(tf_2_contact_frame[6])))
tf_2_contact_frame = frame(toRot(quat_tf_2_contact),
                           vector(constant(tf_2_contact_frame[1]), constant(tf_2_contact_frame[2]), constant(tf_2_contact_frame[3])))

-- contact frame w.r.t. the robot base (depends on the joint positions)
contact_frame = task_frame*tf_2_contact_frame


-- ========================================= FORCE MEASUREMENTS ===================================
-- Wrench measured by the F/T sensor, expressed in FT_sensor_frame with its origin as reference point
raw_wrench = ctx:createInputChannelWrench("wrench_input")

-- 0) Low-pass filter each component of the raw measurement (Butterworth filter, see utilities/filters_etasl.lua).
--    The filter is added to the eTaSL controller: its states are feature variables that start at zero.
--    To avoid that the filtered signal starts at zero, the filter is applied to the change of the signal
--    w.r.t. its initial value, and the initial value is added again (the filter has a static gain of one).
--    filters_etasl is a file of this library (and not of the application ROS2 package), so its directory is added to package.path:
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path
require("filters_etasl")

function low_pass(name, signal)
    if filter_cutoff_frequency <= 0 then
        return signal
    end
    local signal_0 = initial_value(time, signal)
    local omega    = 2*math.pi*filter_cutoff_frequency -- cut-off frequency [rad/s]
    return signal_0 + generate_butter_low_pass(ctx, name, omega, signal - signal_0, filter_order)
end

sensed_wrench = wrench(
    vector(low_pass("filter_Fx", coord_x(force(raw_wrench))),
           low_pass("filter_Fy", coord_y(force(raw_wrench))),
           low_pass("filter_Fz", coord_z(force(raw_wrench)))),
    vector(low_pass("filter_Tx", coord_x(torque(raw_wrench))),
           low_pass("filter_Ty", coord_y(torque(raw_wrench))),
           low_pass("filter_Tz", coord_z(torque(raw_wrench)))))

-- 1) Wrench acting ON the robot through the sensor:
if param.get("wrench_convention") == "exerted_on_environment" then
    wrench_on_robot = -sensed_wrench
else
    wrench_on_robot = sensed_wrench
end

-- 2) Gravity compensation: subtract the weight of the tool (after the sensor). The weight acts
--    along -z of the robot base and is applied at the center of gravity of the tool.
gravity_force_sensor = rotation(inv(FT_sensor_frame))*vector(0, 0, -tool_weight)
gravity_wrench       = wrench(gravity_force_sensor, cross(tool_COG, gravity_force_sensor))
external_wrench      = wrench_on_robot - gravity_wrench

-- 3) Express the external wrench in the contact frame, with the origin of the contact frame as reference point
sensor_wrt_contact     = inv(contact_frame)*FT_sensor_frame
wrench_contact_frame   = ref_point(transform(rotation(sensor_wrt_contact), external_wrench), -origin(sensor_wrt_contact))
force_contact_frame    = force(wrench_contact_frame)


-- ========================================= TASK FRAME ===================================
-- Initial pose of the contact frame (value at the start of the task)
start_pose = initial_value(time, contact_frame)

-- The task frame is the contact frame rotated around its z-axis by an angle alpha0 + alpha:
--   * alpha0: direction of the contact force at the start of the task (the robot is already in contact),
--   * alpha : feature variable that tracks the changes of the direction of the contact force.
alpha0 = initial_value(time, atan2(coord_y(force_contact_frame), coord_x(force_contact_frame)))
alpha  = Variable{context = ctx, name = "alpha", vartype = "feature", initial = 0.0}

taskframe = contact_frame*frame(rot_z(alpha0 + alpha))

-- Inverse of the task frame, frozen at the current time step. Expressing a position with it gives
-- coordinates whose time derivative is the velocity expressed in the (current) task frame axes.
taskframe_inv_c = cached(inv(make_constant(taskframe)))
pos_tf          = taskframe_inv_c*origin(taskframe)

-- Contact force acting ON the robot, expressed in the task frame
force_tf = inv(rot_z(alpha0 + alpha))*force_contact_frame
Fx_tf    = coord_x(force_tf)
Fy_tf    = coord_y(force_tf)


-- =============================== CONSTRAINT SPECIFICATION ==============================
-- Note: a desired velocity v for an expression e is specified with the constraint
--           expr = e - make_constant(v)*time,  K = 0
--       i.e. the time derivative of e must be equal to v.

-- x-direction: normal force control.
-- Model of the contact: moving the task frame along -x (into the contour) increases the force
-- acting on the robot along +x with the contact stiffness: F_x = -contact_stiffness*x.
Constraint{
    context  = ctx,
    name     = "normal_force",
    model    = -contact_stiffness*coord_x(pos_tf),
    meas     = Fx_tf,
    target   = normal_force,
    K        = K_force,
    priority = 2,
    weight   = 1
};

-- y-direction: tangential velocity, linearly increasing during ramp_up_time
ramp = 1.0
if ramp_up_time > 0 then
    ramp = conditional(time - constant(ramp_up_time), constant(1.0), time/constant(ramp_up_time))
end
Constraint{
    context  = ctx,
    name     = "tangential_velocity",
    expr     = coord_y(pos_tf) - make_constant(ramp*tangential_velocity)*time,
    K        = 0,
    priority = 2,
    weight   = 1
};

-- z-direction: keep the initial position along the tool axis
Constraint{
    context  = ctx,
    name     = "keep_height",
    expr     = coord_z(origin(inv(start_pose)*contact_frame)),
    K        = K_hold,
    priority = 2,
    weight   = 1
};

-- Keep the initial orientation of the tool (the robot does not rotate, only the task frame)
Constraint{
    context  = ctx,
    name     = "keep_orientation",
    expr     = rotation(inv(start_pose)*contact_frame),
    K        = K_hold,
    priority = 2,
    weight   = 1
};

-- Rotation of the task frame around z: rotate with an angular velocity proportional to the angle
-- between the x-axis of the task frame and the measured contact force. The task frame then
-- converges to the normal of the contour.
Constraint{
    context  = ctx,
    name     = "track_normal",
    expr     = alpha - make_constant(K_tracking*atan2(Fy_tf, Fx_tf))*time,
    K        = 0,
    priority = 2,
    weight   = 1
};


-- =============================== MONITORS ==============================
-- Travelled path length: feature variable whose velocity is the magnitude of the velocity of the contact frame
travelled_distance = Variable{context = ctx, name = "travelled_distance", vartype = "feature", initial = 0.0}
Constraint{
    context  = ctx,
    name     = "travelled_distance",
    expr     = travelled_distance - make_constant(norm(previous_velocity(time, origin(contact_frame))))*time,
    K        = 0,
    priority = 2,
    weight   = 1
};

Monitor{
    context    = ctx,
    name       = "finish_after_path_length",
    upper      = path_length,
    actionname = "exit",
    expr       = travelled_distance
};


-- =============================== OUTPUTS ==============================
roll_tf, pitch_tf, yaw_tf = getRPY(rotation(contact_frame))

ctx:setOutputExpression("x_tf"      ,coord_x(origin(contact_frame)))
ctx:setOutputExpression("y_tf"      ,coord_y(origin(contact_frame)))
ctx:setOutputExpression("z_tf"      ,coord_z(origin(contact_frame)))
ctx:setOutputExpression("roll_tf"   ,roll_tf)
ctx:setOutputExpression("pitch_tf"  ,pitch_tf)
ctx:setOutputExpression("yaw_tf"    ,yaw_tf)

-- external wrench acting on the robot, expressed in the contact frame
ctx:setOutputExpression("x_force"   ,coord_x(force(wrench_contact_frame)))
ctx:setOutputExpression("y_force"   ,coord_y(force(wrench_contact_frame)))
ctx:setOutputExpression("z_force"   ,coord_z(force(wrench_contact_frame)))
ctx:setOutputExpression("x_torque"  ,coord_x(torque(wrench_contact_frame)))
ctx:setOutputExpression("y_torque"  ,coord_y(torque(wrench_contact_frame)))
ctx:setOutputExpression("z_torque"  ,coord_z(torque(wrench_contact_frame)))

-- contact force expressed in the task frame (normal and tangential component)
ctx:setOutputExpression("normal_force"       ,Fx_tf)
ctx:setOutputExpression("tangential_force"   ,Fy_tf)
ctx:setOutputExpression("normal_angle"       ,alpha0 + alpha)
ctx:setOutputExpression("travelled_distance" ,travelled_distance)
ctx:setOutputExpression("tangential_speed"   ,norm(previous_velocity(time, origin(contact_frame))))
