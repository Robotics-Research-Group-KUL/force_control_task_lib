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
task_description = [[Admittance control: the robot is compliant to external forces and torques measured by a force/torque sensor, e.g. to guide it by hand. Along each axis of task_frame the robot moves with a velocity proportional to the external force: K_force*F/K_t [m/s] for translations (if activate_linear) and K_torque*T/K_r [rad/s] for rotations (if activate_angular); i.e. it behaves as a damper with damping K_t/K_force [Ns/m] and K_r/K_torque [Nms/rad] and does not return to its initial pose. Non-activated motions keep their initial value. The measured wrench is read from the input channel named by wrench_input_channel (default "wrench_input") (type wrench, expressed in FT_sensor_frame with its origin as reference point), provided by the robot driver/simulator (e.g. MuJoCo) or by a WrenchInputHandler; it is converted to the wrench acting on the robot (see wrench_convention), passed through a dead zone, compensated for the tool weight and expressed in task_frame with its origin as reference point. The task has no monitor: it runs until it is stopped from outside. Outputs: time, position of task_frame (x_tcp, y_tcp, z_tcp), its pose (tf) and the raw input force (x_force, y_force, z_force, as received on wrench_input).
Gravity compensation (tool_weight > 0): it assumes that the force/torque sensor was tared with the tool completely vertical, pointing down (the z axis of FT_sensor_frame along gravity, -z of the robot base frame); the tool weight measured in other orientations of the tool is then compensated.]]

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="K_t", description="Translational stiffness [N/m] used as model of the admittance along the x, y and z axes of task_frame: the velocity is K_force*F/K_t, with F the external force [N]. Higher values make the robot move slower for the same force (it feels heavier); lower values make it more compliant. Must be positive", default = 2000, required=true}),
    reqs.params.scalar({name="K_r", description="Rotational stiffness [Nm/rad] used as model of the admittance around the x, y and z axes of task_frame: the angular velocity is K_torque*T/K_r, with T the external torque [Nm] w.r.t. the origin of task_frame. Higher values make the robot rotate slower for the same torque; lower values make it more compliant. Must be positive. Only used if activate_angular is true", default = 200, required=true}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (from the robot model) that is controlled, e.g. the tool center point. The admittance is applied along/around its axes and the torques are taken w.r.t. its origin", default = "tcp_frame", required=false}),
    reqs.params.string({name="FT_sensor_frame", description="Name of the robot frame (from the robot model) of the force/torque sensor. The measured wrench of the input channel (wrench_input_channel) is expressed in this frame, with its origin as reference point", default = "FT_sensor_frame", required=true}),
    reqs.params.enum({name="wrench_convention", type=reqs.enum_types.string, default="exerted_on_robot",
                            description="Sign convention of the wrench received on the input channel (wrench_input_channel). exerted_on_environment: it is the wrench that the robot exerts on the environment (e.g. the MuJoCo simulator); it is negated to obtain the wrench acting on the robot. exerted_on_robot: it is the wrench that the environment (e.g. a person guiding the robot) exerts on the robot and is used as is. The robot moves along the wrench acting on it", required=false, accepted_vals = {"exerted_on_environment","exerted_on_robot"}}),
    reqs.params.bool({name="activate_linear", description="If true, the translations of task_frame are compliant to the external forces (admittance with K_t). If false, the position of task_frame is kept at its initial value", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the rotations of task_frame are compliant to the external torques (admittance with K_r). If false, the orientation of task_frame is kept at its initial value", default = false, required=true}),
    reqs.params.scalar({name="force_threshold", description="Dead zone [N] applied to each force component of the measured wrench (in FT_sensor_frame): components with an absolute value below it are set to zero and larger ones are reduced by it. It avoids drift due to sensor noise and offsets. 0 disables the dead zone", default = 0.5, required=false}),
    reqs.params.scalar({name="torque_threshold", description="Dead zone [Nm] applied to each torque component of the measured wrench (in FT_sensor_frame): components with an absolute value below it are set to zero and larger ones are reduced by it. 0 disables the dead zone", default = 0.05, required=false}),
    reqs.params.scalar({name="tool_weight", description="Weight [N] (gravity force, not mass) of the tool mounted after the force/torque sensor, applied at tool_COG, with gravity along -z of the robot base frame. The gravity compensation assumes that the force/torque sensor was tared with the tool completely vertical, pointing down (the z axis of FT_sensor_frame along gravity): it removes the change of the tool weight measured by the sensor when the tool is in another orientation. 0 disables the gravity compensation (e.g. when the tool weight is negligible, or the simulator does not measure it)", default = 0.0, required=true}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, 
                            description="Position [m] of the center of gravity of the tool (everything mounted after the force/torque sensor), as [x,y,z] expressed in FT_sensor_frame. Used for the gravity compensation together with tool_weight", required=true, minItems = 3, maxItems = 3}),
    reqs.params.scalar({name="K_force", description="Feedback gain [1/s] of the translational admittance constraints (activate_linear true): the velocity is K_force*F/K_t. Higher values make the robot move faster for the same force", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_force", description="Weight [-] of the translational admittance constraints relative to the other constraints of the same priority. Only matters when the constraints conflict", default = 1, required=false, minimum = 0.0}),
    reqs.params.scalar({name="K_torque", description="Feedback gain [1/s] of the rotational admittance constraints (activate_angular true): the angular velocity is K_torque*T/K_r. Higher values make the robot rotate faster for the same torque", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_torque", description="Weight [-] of the rotational admittance constraints relative to the other constraints of the same priority. Only matters when the constraints conflict", default = 1, required=false, minimum = 0.0}),
    reqs.params.scalar({name="K_keep_position", description="Feedback gain [1/s] of the constraint that keeps the initial position of task_frame when activate_linear is false. Higher values correct deviations faster", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_keep_position", description="Weight [-] of the constraint that keeps the initial position of task_frame (activate_linear false) relative to the other constraints of the same priority", default = 1, required=false, minimum = 0.0}),
    reqs.params.scalar({name="K_keep_orientation", description="Feedback gain [1/s] of the constraint that keeps the initial orientation of task_frame when activate_angular is false. Higher values correct deviations faster", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="weight_keep_orientation", description="Weight [-] of the constraint that keeps the initial orientation of task_frame (activate_angular false) relative to the other constraints of the same priority", default = 1, required=false, minimum = 0.0}),
    reqs.params.string({name="wrench_input_channel", description="Name of the input channel (type wrench) from which the measured wrench of the force/torque sensor is read: the varname of the input handler that provides it (e.g. a WrenchInputHandler), or the channel fed directly by the robot driver/simulator (name_expr_cartesian_wrench of the MuJoCo simulator). Change it to use another sensor or input handler", default = "wrench_input", required=false}),
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
wrench_input   = ctx:createInputChannelWrench(param.get("wrench_input_channel"))
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
        K = constant(param.get("K_force")),
        priority = 2,
        weight = constant(param.get("weight_force")),
    };

    Constraint{
        context=ctx,
        name="follow_force_y",
        model = -K_t*coord_y(origin(task_frame_inst)),
        meas = Fy,
        target = 0,
        K = constant(param.get("K_force")),
        priority = 2,
        weight = constant(param.get("weight_force")),
    };

    Constraint{
        context=ctx,
        name="follow_force_z",
        model = -K_t*coord_z(origin(task_frame_inst)),
        meas = Fz,
        target = 0,
        K = constant(param.get("K_force")),
        priority = 2,
        weight = constant(param.get("weight_force")),
    };

else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(task_frame_inst),
        target  = initial_value(time, origin(task_frame_inst)),
        K       = param.get("K_keep_position"),
        weight  = param.get("weight_keep_position"),
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
        K = constant(param.get("K_torque")),
        priority = 2,
        weight = constant(param.get("weight_torque")),
    };

    Constraint{
        context=ctx,
        name="follow_torque_y",
        model = -K_r*coord_y(getRotVec(rotation(task_frame_inst))),
        meas = Ty,
        target = 0,
        K = constant(param.get("K_torque")),
        priority = 2,
        weight = constant(param.get("weight_torque")),
    };

    Constraint{
        context=ctx,
        name="follow_torque_z",
        model = -K_r*coord_z(getRotVec(rotation(task_frame_inst))),
        meas = Tz,
        target = 0,
        K = constant(param.get("K_torque")),
        priority = 2,
        weight = constant(param.get("weight_torque")),
    };
else
    intial_rot=initial_value(time,rotation(task_frame))
    Constraint {
        context         = ctx,
        name            = "keep_rot",
        expr            = rotation(task_frame)*inv(intial_rot),
        weight          = param.get("weight_keep_orientation"),
        priority        = 2,
        K               = param.get("K_keep_orientation")
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