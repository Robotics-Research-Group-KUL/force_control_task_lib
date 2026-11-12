--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Authors: Santiago Iregui, Federico Ulloa
--  email: <santiago.iregui@kuleuven.be>
-- 
-- Code made based on Cristian Vergara's and Erwin Aertbeliën's code. 
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

-- ========================================= PARAMETERS ===================================
task_description = "This task specification allows to control the robot with an admittance controller using force/torque sensor data."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="K_t", description="Translational stiffness [N/m]", default = 2000, required=true}),
    reqs.params.scalar({name="K_r", description="Rotational stiffness [Nm/rad]", default = 200, required=true}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.string({name="FT_sensor_frame", description="Name of frame at the force/torque sensor", default = "FT_sensor_frame", required=true}),
    reqs.params.enum({name="wrench_convention", type=reqs.enum_types.string, default="exerted_on_robot",
                            description="Sign convention of the measured wrench: wrench that the robot exerts on the environment (e.g. MuJoCo simulator) or wrench that the environment exerts on the robot", required=false, accepted_vals = {"exerted_on_environment","exerted_on_robot"}}),
    reqs.params.bool({name="activate_linear", description="If true, the linear velocities of the joystick will control the linear velocities of the defined frame", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular velocities of the joystick will control the angular velocities of the defined frame", default = false, required=true}),
    reqs.params.scalar({name="force_threshold", description="Force dead zone [N]", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Torque dead zone [Nm]", default = 0.05, required=false}),
    reqs.params.scalar({name="tool_weight", description="Weight of the tool attached to the end-effector [N]", default = 0.0, required=true}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Array with the center of gravity of the tool w.r.t FT_sensor_frame [m]", required=true, minItems = 3, maxItems = 3}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    param.get("FT_sensor_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints

-- ======================================== FRAMES ========================================
task_frame = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))

-- ========================================= Variables coming from topic input handlers ===================================
-- ========================================= Variables coming from topicinput handlers ===================================
wrench_input   = ctx:createInputChannelWrench("wrench_input")
-- wrench_input = wrench(vector(0,0,-5),vector(0,0,0))

K_t = param.get("K_t")
K_r = param.get("K_r")
force_threshold = param.get("force_threshold")
torque_threshold = param.get("torque_threshold")
tool_weight = param.get("tool_weight")
tool_COG = param.get("tool_COG")

tool_COG_x = constant(tool_COG[1])
tool_COG_y = constant(tool_COG[2])
tool_COG_z = constant(tool_COG[3])

-- The admittance follows the wrench that the environment (e.g. a person guiding the robot) exerts on the robot
if param.get("wrench_convention") == "exerted_on_environment" then
    wrench_on_robot = -wrench_input
else
    wrench_on_robot = wrench_input
end

-- =============================== TRANSFORM WRENCH TO TASK FRAME==============================
Fx = coord_x(force(wrench_on_robot))
Fy = coord_y(force(wrench_on_robot))
Fz = coord_z(force(wrench_on_robot))
Tx = coord_x(torque(wrench_on_robot))
Ty = coord_y(torque(wrench_on_robot))
Tz = coord_z(torque(wrench_on_robot))

-- ===================================== TRANSFORM WRENCH TO TASK FRAME================================


function dead_zone(signal,dead_val)
    signal_dead_zone = conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
    return signal_dead_zone
end



Fx_dead_zone = dead_zone(Fx,force_threshold)
Fy_dead_zone = dead_zone(Fy,force_threshold)
Fz_dead_zone = dead_zone(Fz,force_threshold)
Tx_dead_zone = dead_zone(Tx,torque_threshold)
Ty_dead_zone = dead_zone(Ty,torque_threshold)
Tz_dead_zone = dead_zone(Tz,torque_threshold)

-- =============================== GRAVITY COMPENSATION==============================

-- WARNING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This version works in the UR10e robot, but not in Maira or IIWA
-- d_g = vector(0,0,-1) -- wrt to the base frame
-- FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

-- -- This is the wrench removed by the taring and that needs to be compensated (Assuming it was tared aligned with the gravity vector)
-- virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))
-- virtual_wrench_wrt_FT_frame = transform(rotation(FT_sensor_frame), virtual_wrench_wrt_base_frame)

-- -- This is the wrench caused by the tool weight that needs to be compensated in the FT_sensor_frame
-- -- Rotate the wrench to the FT_sensor_frame orientation
-- wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))

-- -- Translate the wrench to the FT_sensor_frame
-- wrench_FT_frame = ref_point(wrench_cog_ftframe, origin(inv(FT_sensor_frame_to_cog)))
-- wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame + virtual_wrench_wrt_FT_frame


-- WARNING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! This version works in the in Maira or IIWA but not in UR10e
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(tool_COG_x, tool_COG_y, tool_COG_z))

virtual_wrench_wrt_base_frame = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog),d_g*tool_weight))

wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))

wrench_dead_zone = wrench(vector(Fx_dead_zone,Fy_dead_zone,Fz_dead_zone),vector(Tx_dead_zone,Ty_dead_zone,Tz_dead_zone)) - wrench_FT_frame - virtual_wrench_wrt_base_frame


-- =============================== TRANSLATE FT TO TASK_FRAME==============================

wrench_task_frame   = ref_point(transform(rotation(inv(task_frame)*FT_sensor_frame),wrench_dead_zone) , -origin(inv(task_frame)*FT_sensor_frame))

Fx = coord_x(force(wrench_task_frame))
Fy = coord_y(force(wrench_task_frame))
Fz = coord_z(force(wrench_task_frame))
Tx = coord_x(torque(wrench_task_frame))
Ty = coord_y(torque(wrench_task_frame))
Tz = coord_z(torque(wrench_task_frame))

-- =============================== INSTANTANEOUS FRAME ==============================
task_frame_inst = inv(make_constant(task_frame))*task_frame

-- =============================== ADMITTANCE CONTROLLER ==============================
-- Force constraints
if param.get("activate_linear") then
    Constraint{
        context=ctx,
        name="follow_force_x",
        model = -K_t*coord_x(origin(task_frame_inst)),
        meas = Fx,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };

    Constraint{
        context=ctx,
        name="follow_force_y",
        model = -K_t*coord_y(origin(task_frame_inst)),
        meas = Fy,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };

    Constraint{
        context=ctx,
        name="follow_force_z",
        model = -K_t*coord_z(origin(task_frame_inst)),
        meas = Fz,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };

else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(task_frame_inst),
        target  = initial_value(time, origin(task_frame_inst)),
        K       = 4,
        weight  = 1,
        priority= 2
    };

end

-- Torque constraints
if param.get("activate_angular") then
    Constraint{
        context=ctx,
        name="follow_torque_x",
        model = -K_r*coord_x(getRotVec(rotation(task_frame_inst))),
        meas = Tx,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };

    Constraint{
        context=ctx,
        name="follow_torque_y",
        model = -K_r*coord_y(getRotVec(rotation(task_frame_inst))),
        meas = Ty,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };

    Constraint{
        context=ctx,
        name="follow_torque_z",
        model = -K_r*coord_z(getRotVec(rotation(task_frame_inst))),
        meas = Tz,
        target = 0,
        K = constant(4),
        priority = 2,
        weight = constant(1),
    };
else
    intial_rot=initial_value(time,rotation(task_frame))
    Constraint {
        context         = ctx,
        name            = "keep_rot",
        expr            = rotation(task_frame)*inv(intial_rot),
        weight          = 1,
        priority        = 2,
        K               = 4
    };
end
-- -- Constant orientation
-- intial_rot=initial_value(time,rotation(task_frame))
-- Constraint {
--     context         = ctx,
--     name            = "keep_rot",
--     expr            = rotation(task_frame)*inv(intial_rot),
--     weight          = 1,
--     priority        = 2,
--     K               = 4
-- };


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tcp",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tcp",coord_z(origin(task_frame)))
ctx:setOutputExpression("tf",task_frame)

ctx:setOutputExpression("x_force",coord_x(force(wrench_input)))
ctx:setOutputExpression("y_force",coord_y(force(wrench_input)))
ctx:setOutputExpression("z_force",coord_z(force(wrench_input)))






-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(tf)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)