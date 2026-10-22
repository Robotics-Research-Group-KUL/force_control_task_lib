--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
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
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")

task_description = "This task specification implements a guarded motion in the world frame or task frame. The robot will move along the specified axes of the specified frame until a contact force is detected."


-- ========================================= PARAMETERS ===================================
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s]", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="time_delay", description="Time delay [s] to avoid activating the monitor at the beginning, where forces can be present due to discrete transitions of controllers", default = 0.0, required=true, minimum = 0.0}),
    -- reqs.params.scalar({name="contact_force", description="Contact force [N]", default = 1.0, required=true}),
    reqs.params.array({name="contact_forces", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Contact forces [N] array in [x,y,z]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="force_monitor", description="Force value (in the guarded axis direction) which will trigger the monitor", default = 3.0, required=true}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
    reqs.params.array({name="tf_2_insframe", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, 
                            description="Transformation from the task_frame to the insertion_frame in [x,y,z,qx,qy,qz,qw]", required=true, minItems = 7, maxItems = 7}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=true}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame where the forces and torques a measured", default = "FT_sensor_frame", required=true}),
    reqs.params.array({name="K_F", type=reqs.array_types.number, default={2700, 2700, 2700},
                            description="Array with the diagonal elements of the translational stiffness matrix [N/m]", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.array({name="K_T", type=reqs.array_types.number, default={200, 200, 200},
                            description="Array with the diagonal elements of the rotational stiffness matrix [Nm/rad]", required=true, minimum = 0.0, minItems = 3, maxItems = 3}),
    reqs.params.enum({name="axis", type=reqs.enum_types.string, default="z", description="Axis in which the guarded motion will take place", required=true, accepted_vals = {"x","y","z"}}),
    reqs.params.enum({name="guarded_motion_frame", type=reqs.enum_types.string, default="task_frame", description="Frame in which the guarded motion will take place", required=true, accepted_vals = {"task_frame","world_frame"}}),

})



-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))
guarded_motion_frame = param.get("guarded_motion_frame")

-- ========================================= FUNCTIONS ===================================
function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end

 
-- ========================================= PARAMETERS ===================================
maxvel              = constant(param.get("maxvel"))
contact_forces      = param.get("contact_forces")
force_monitor        = constant(param.get("force_monitor"))
force_threshold     = constant(param.get("force_threshold"))
time_delay          = constant(param.get("time_delay"))
torque_threshold    = constant(param.get("torque_threshold"))
tool_COG            = param.get("tool_COG")
tool_weight         = constant(param.get("tool_weight"))
K_F                 = param.get("K_F")
K_T                 = param.get("K_T")
tf_2_insframe     = param.get("tf_2_insframe")

axis                = param.get("axis")


tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

target_force_x = constant(contact_forces[1])
target_force_y = constant(contact_forces[2])
target_force_z = constant(contact_forces[3])

K_F_x = constant(K_F[1])
K_F_y = constant(K_F[2])
K_F_z = constant(K_F[3])

K_T_x = constant(K_T[1])
K_T_y = constant(K_T[2])
K_T_z = constant(K_T[3])

tcp_tf_x = constant(tf_2_insframe[1])
tcp_tf_y = constant(tf_2_insframe[2])
tcp_tf_z = constant(tf_2_insframe[3])
q_i     = constant(tf_2_insframe[4])
q_j     = constant(tf_2_insframe[5])
q_k     = constant(tf_2_insframe[6])
q_real  = constant(tf_2_insframe[7])

-- compute orientation from quaternion
quat_tf_2_insframe = quaternion(q_real,vector(q_i,q_j,q_k))
tf_2_insframe = frame(toRot(quat_tf_2_insframe), vector(tcp_tf_x,tcp_tf_y,tcp_tf_z))

-- ========================================= Variables coming from topic input handlers ===================================
sensed_wrench   = ctx:createInputChannelWrench("wrench_input")

-- =============================== TRANSFORM WRENCH TO TASK FRAME ==============================
Fx = coord_x(force(sensed_wrench))
Fy = coord_y(force(sensed_wrench))
Fz = coord_z(force(sensed_wrench))
Tx = coord_x(torque(sensed_wrench))
Ty = coord_y(torque(sensed_wrench))
Tz = coord_z(torque(sensed_wrench))

Fx_dead_zone = dead_zone(Fx,force_threshold)
Fy_dead_zone = dead_zone(Fy,force_threshold)
Fz_dead_zone = dead_zone(Fz,force_threshold)
Tx_dead_zone = dead_zone(Tx,torque_threshold)
Ty_dead_zone = dead_zone(Ty,torque_threshold)
Tz_dead_zone = dead_zone(Tz,torque_threshold)
-- =============================== GRAVITY COMPENSATION ==============================
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

virtual_wrench = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench

-- =============================== TRANSLATE FT TO TASK_FRAME ==============================
task_frame_c = task_frame*tf_2_insframe

wrench_task_frame   = ref_point(transform(rotation(inv(task_frame_c)*FT_sensor_frame), wrench_dead_zone) , -origin(inv(task_frame_c)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== INSTANTANEOUS FRAME ==============================
if guarded_motion_frame == "task_frame" then
    task_frame_inst = inv(make_constant(task_frame_c))*task_frame_c -- control in the task frame
else
    task_frame_inst = task_frame --control in the world frame
end

-- =============================== INITIAL VALUES ==============================
startpose = initial_value(time, task_frame_c)
start_pose_diff  = inv(startpose)*task_frame_c

-- =============================== CONSTRAINT SPECIFICATION ==============================
if axis=="x" then
    contact_force_guarded_axis = target_force_x
elseif axis=="y" then
    contact_force_guarded_axis = target_force_y
elseif axis=="z" then
    contact_force_guarded_axis = target_force_z
end

print("================================")
print("---------------++++++++++++++++The contact force (controled) is: ", contact_force_guarded_axis)
print("---------------++++++++++++++++The force monitor is: ", force_monitor)
print("================================")


Constraint{
	context=ctx,
	name="follow_force_x",
	model = -K_F_x*coord_x(origin(task_frame_inst)),
	meas = Fx,
	target = target_force_x,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
	context=ctx,
	name="follow_force_y",
	model = -K_F_y*coord_y(origin(task_frame_inst)),
	meas = Fy,
	target = target_force_y,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
	context=ctx,
	name="follow_force_z",
	model = -K_F_z*coord_z(origin(task_frame_inst)),
	meas = Fz,
	target = target_force_z,
	K = constant(4),
	priority = 2,
	weight = constant(1),
};

Constraint{
    context = ctx,
    name    = "constant_orientation",
    expr    = rotation(start_pose_diff),
    K       = 4,
    weight  = 1,
    priority= 2
};


time_start = conditional(time-time_delay, 1, 0)

if axis=="x" then
    Constraint{
        context = ctx,
        name    = "x_max_velocity",
        expr    = coord_x(origin(task_frame_inst)),
        target_upper = maxvel*time,
        target_lower = -maxvel*time,
        K       = 0,
        weight  = 10,
        priority= 1
    };

    Monitor{context=ctx,
        name='finish_force_x',
        upper=1.0,
        actionname='exit',
        expr=time_start*abs(Fx)/abs(force_monitor)
    };
elseif axis=="y" then
    Constraint{
        context = ctx,
        name    = "y_max_velocity",
        expr    = coord_y(origin(task_frame_inst)),
        target_upper = maxvel*time,
        target_lower = -maxvel*time,
        K       = 0,
        weight  = 10,
        priority= 1
    };

    Monitor{context=ctx,
        name='finish_force_y',
        upper=1.0,
        actionname='exit',
        expr=time_start*abs(Fy)/abs(force_monitor)
    };
elseif axis=="z" then
    Constraint{
        context = ctx,
        name    = "z_max_velocity",
        expr    = coord_z(origin(task_frame_inst)),
        target_upper = maxvel*time,
        target_lower = -maxvel*time,
        K       = 0,
        weight  = 10,
        priority= 1
    };

    Monitor{context=ctx,
            name='finish_force_z',
            upper=1.0,
            actionname='exit',
            expr=time_start*abs(Fz)/abs(force_monitor)
        };
end


-- require("velocities")

-- traveled_distance_z = Variable{
-- 	context=ctx,
-- 	name='traveled_distance_z',
-- 	vartype ='feature',
-- 	initial = 0
-- };

-- Constraint{
-- 	context=ctx,
-- 	name="distance_integrator",
-- 	expr=traveled_distance_z - make_constant(previous_velocity(time, coord_z(origin(task_frame_c))))*time,
-- 	K = 0,
-- 	priority = 2,
-- 	weight = constant(1),
-- };

-- monitor_dist_z = Monitor{
-- 						context=ctx,
-- 						name='finish_after_distance',
-- 						upper = 0.025,
-- 						actionname='event',
-- 						expr = abs(traveled_distance_z),
-- 						argument = "e_max_z_dist_reached"
-- 					};




roll_tf, pitch_tf, yaw_tf = getRPY(rotation(task_frame_c))


ctx:setOutputExpression("x_tf"		,coord_x(origin(task_frame_c)))
ctx:setOutputExpression("y_tf"		,coord_y(origin(task_frame_c)))
ctx:setOutputExpression("z_tf"		,coord_z(origin(task_frame_c)))
ctx:setOutputExpression("roll_tf"	,roll_tf)
ctx:setOutputExpression("pitch_tf"  ,pitch_tf)
ctx:setOutputExpression("yaw_tf"	,yaw_tf)

ctx:setOutputExpression("x_force"      ,Fx)
ctx:setOutputExpression("y_force"      ,Fy)
ctx:setOutputExpression("z_force"      ,Fz)
ctx:setOutputExpression("x_torque"      ,Tx)
ctx:setOutputExpression("y_torque"      ,Ty)
ctx:setOutputExpression("z_torque"      ,Tz)

-- ctx:setOutputExpression("tcp_frame"   , tcp_frame)
ctx:setOutputExpression("task_frame_c"  ,task_frame_c)