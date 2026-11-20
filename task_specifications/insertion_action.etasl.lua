--  Copyright (c) 2026 KU Leuven, Belgium
--
--  Authors: Santiago Iregui, Federico Ulloa
--  email: <santiago.iregui@kuleuven.be>
--
--  Based on the methodology of: F. Ulloa Rios, S. Iregui, E. Aertbeliën, W. Decré, H. Bruyninckx, "Robust
--  Force-Controlled Robotic Insertion Skills for Geometrically Diverse Parts Under Uncertainty", CASE 2026.
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
reqs = require("task_requirements")

-- ========================================= PARAMETERS ===================================
task_description = [[Insertion action: one step of a force-controlled insertion skill that reduces one degree of freedom (DoF) of the insertion object (held by the robot) w.r.t. the housing object, by a guarded motion until a contact transition is detected (methodology of "Robust Force-Controlled Robotic Insertion Skills for Geometrically Diverse Parts Under Uncertainty", Ulloa Rios et al., CASE 2026). An insertion skill is a sequence of such actions, one per reduced DoF.
The action is specified at the control frame {f_i} (feature_frame, attached to the insertion object at a mating feature, e.g. a corner of a peg). The 6 DoFs are: the translations x, y, z along the axes of the frame {h f_i}, with the origin of {f_i} and the orientation of the housing (housing_orientation), and the rotations rx, ry, rz around the axes of {f_i}. Each DoF is either force controlled (force_controlled = 1) or velocity controlled (0):
- force controlled: admittance towards the target force/torque of target_wrench (Eq. 1 of the paper): velocity = -K_force*(target - measured)/K_t (or /K_r for rotations), limited to maxvel (maxvel_rot). A non-zero target keeps a contact (or, for the reduced DoF, moves towards a new one); a zero target lets the object follow the contact forces (attraction regions);
- velocity controlled: zero velocity (the current offset is kept).
target_wrench is the superposition of the target wrenches of all the mating features, expressed at {f_i}: e.g. a force -F along z at another feature at position r (in {f_i}) adds the force -F along z and the torque r x (0,0,-F). Its sign convention is the wrench exerted by the environment on the insertion object (e.g. pushing with F along the insertion direction z: target -F along z).
The task finishes (monitor action "exit") when, after time_delay, the measured force/torque of the reduced DoF (reduced_dof) reaches monitor_fraction times its target (a contact transition): reduced_dof must be force controlled with a non-zero target.
The measured wrench is read from the input channel named by wrench_input_channel (default "wrench_input") (type wrench, expressed in FT_sensor_frame with its origin as reference point), provided by the robot driver/simulator (e.g. MuJoCo) or by a WrenchInputHandler; it is converted to the wrench exerted on the robot (see wrench_convention), passed through a dead zone, compensated for the tool weight and expressed at {f_i}. Outputs: time, the position of {f_i} (x_f, y_f, z_f), the frames feature_frame ({f_i}) and feature_frame_housing ({h f_i}), and the processed wrench (x_force, y_force, z_force along the axes of {h f_i}; x_torque, y_torque, z_torque around the axes of {f_i}).
Gravity compensation (tool_weight > 0): it assumes that the force/torque sensor was tared with the tool completely vertical, pointing down (the z axis of FT_sensor_frame along gravity, -z of the robot base frame); the tool weight measured in other orientations of the tool is then compensated.]]

param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of the robot frame (from the robot model) that is controlled, e.g. the tool center point. The control frame {f_i} is defined w.r.t. this frame by feature_frame", default = "tcp_frame", required=true}),
    reqs.params.array({name="feature_frame", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                            description="Pose of the control frame {f_i} (at the mating feature of the insertion object, e.g. a corner of the peg, with z along the insertion direction) w.r.t. task_frame, as [x,y,z,qx,qy,qz,qw] (position in m, unit quaternion with the real part qw last), expressed in task_frame. The torques of target_wrench are w.r.t. its origin and around its axes", required=true, minItems = 7, maxItems = 7}),
    reqs.params.array({name="housing_orientation", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 1.0},
                            description="Orientation of the housing feature frame {f_h} (the mating feature of the housing, e.g. the corner of the hole, with z along the insertion direction) w.r.t. the robot base (world) frame, as a unit quaternion [qx,qy,qz,qw] (real part last). The translations and the forces of target_wrench are along the axes of the frame {h f_i}: origin of {f_i}, this orientation", required=true, minItems = 4, maxItems = 4}),
    reqs.params.array({name="target_wrench", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                            description="Target wrench [Fx,Fy,Fz,Tx,Ty,Tz] ([N] along the axes of {h f_i}, [Nm] around the axes of {f_i}, w.r.t. the origin of {f_i}) of the force-controlled DoFs: the superposition of the target wrenches of all the mating features, in the sign convention of the wrench exerted by the environment on the insertion object (e.g. -F along z pushes with F along the insertion direction). The elements of velocity-controlled DoFs are ignored", required=true, minItems = 6, maxItems = 6}),
    reqs.params.array({name="force_controlled", type=reqs.array_types.number, default={0, 0, 1, 0, 0, 0},
                            description="Control mode of each DoF [x,y,z,rx,ry,rz]: 1 force controlled (admittance towards the element of target_wrench; 0 as target makes the DoF compliant), 0 velocity controlled (zero velocity: the current offset is kept)", required=true, minItems = 6, maxItems = 6}),
    reqs.params.enum({name="reduced_dof", type=reqs.enum_types.string, default="z", accepted_vals = {"x","y","z","rx","ry","rz"},
                            description="DoF reduced by this action: the force/torque of this DoF is monitored and the task finishes when it reaches monitor_fraction times its target in target_wrench (the contact transition). It must be force controlled, with a non-zero target", required=true}),
    reqs.params.scalar({name="monitor_fraction", description="Fraction delta in (0,1] of the target of the reduced DoF that finishes the task: the task exits when the measured force/torque of reduced_dof divided by its target exceeds it (after time_delay). Lower values detect the contact earlier; 1 may never be reached (the admittance approaches its target asymptotically)", default = 0.9, required=false, minimum = 0.0, maximum = 1.0}),
    reqs.params.scalar({name="time_delay", description="Time [s] after the start of the task during which the monitor is disabled, to ignore transients of the forces at the start of the action. 0 activates it immediately", default = 0.3, required=false, minimum = 0.0}),
    reqs.params.scalar({name="K_t", description="Translational stiffness [N/m] used as model of the admittance of the force-controlled translations: the velocity is K_force*(measured force - target)/K_t. Lower values make the guarded motions faster (more velocity for the same force error), higher values slower and stiffer in contact", default = 2000, required=true, minimum = 0.0}),
    reqs.params.scalar({name="K_r", description="Rotational stiffness [Nm/rad] used as model of the admittance of the force-controlled rotations: the angular velocity is K_force*(measured torque - target)/K_r. Lower values make the rotations faster, higher values slower and stiffer in contact", default = 20, required=true, minimum = 0.0}),
    reqs.params.scalar({name="K_force", description="Feedback gain [1/s] of the admittance constraints (k_c of Eq. 1 of the paper, related to the bandwidth of the force control). Higher values react faster, but too high values cause oscillations in contact", default = 4, required=false, minimum = 0.0}),
    reqs.params.scalar({name="maxvel", description="Maximum absolute velocity [m/s] of each force-controlled translation", default = 0.02, required=false, minimum = 0.0}),
    reqs.params.scalar({name="maxvel_rot", description="Maximum absolute angular velocity [rad/s] of each force-controlled rotation", default = 0.1, required=false, minimum = 0.0}),
    reqs.params.string({name="FT_sensor_frame", description="Name of the robot frame (from the robot model) of the force/torque sensor. The measured wrench of the input channel (wrench_input_channel) is expressed in this frame, with its origin as reference point", default = "FT_sensor_frame", required=true}),
    reqs.params.enum({name="wrench_convention", type=reqs.enum_types.string, default="exerted_on_robot", accepted_vals = {"exerted_on_environment","exerted_on_robot"},
                            description="Sign convention of the wrench received on the input channel (wrench_input_channel). exerted_on_environment: it is the wrench that the robot exerts on the environment (e.g. the MuJoCo simulator); it is negated. exerted_on_robot: it is the wrench that the environment exerts on the robot and is used as is", required=false}),
    reqs.params.scalar({name="force_threshold", description="Dead zone [N] applied to each force component of the measured wrench (in FT_sensor_frame): components with an absolute value below it are set to zero and larger ones are reduced by it. It suppresses sensor noise and offsets. 0 disables the dead zone", default = 0.5, required=false, minimum = 0.0}),
    reqs.params.scalar({name="torque_threshold", description="Dead zone [Nm] applied to each torque component of the measured wrench (in FT_sensor_frame): components with an absolute value below it are set to zero and larger ones are reduced by it. 0 disables the dead zone", default = 0.01, required=false, minimum = 0.0}),
    reqs.params.scalar({name="tool_weight", description="Weight [N] (gravity force, not mass) of the tool mounted after the force/torque sensor, applied at tool_COG, with gravity along -z of the robot base frame. The gravity compensation assumes that the force/torque sensor was tared with the tool completely vertical, pointing down (the z axis of FT_sensor_frame along gravity): it removes the change of the tool weight measured by the sensor when the tool is in another orientation. 0 disables the gravity compensation", default = 0.0, required=false, minimum = 0.0}),
    reqs.params.array({name="tool_COG", type=reqs.array_types.number, default={0.0, 0.0, 0.0},
                            description="Position [m] of the center of gravity of the tool (everything mounted after the force/torque sensor), as [x,y,z] expressed in FT_sensor_frame. Used for the gravity compensation together with tool_weight", required=false, minItems = 3, maxItems = 3}),
    reqs.params.string({name="wrench_input_channel", description="Name of the input channel (type wrench) from which the measured wrench of the force/torque sensor is read: the varname of the input handler that provides it (e.g. a WrenchInputHandler), or the channel fed directly by the robot driver/simulator (name_expr_cartesian_wrench of the MuJoCo simulator)", default = "wrench_input", required=false}),
})

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"),
    param.get("FT_sensor_frame"),
})
robot_joints = robot.robot_joints

-- ======================================== FRAMES ========================================
task_frame      = robot.getFrame(param.get("task_frame"))
FT_sensor_frame = robot.getFrame(param.get("FT_sensor_frame"))

local function pose_from_array(a)       -- [x,y,z,qx,qy,qz,qw] -> frame
    return frame(toRot(quaternion(constant(a[7]), vector(constant(a[4]), constant(a[5]), constant(a[6])))),
                 vector(constant(a[1]), constant(a[2]), constant(a[3])))
end
local q_h = param.get("housing_orientation")
R_world_housing = toRot(quaternion(constant(q_h[4]), vector(constant(q_h[1]), constant(q_h[2]), constant(q_h[3]))))

-- {f_i}: control frame at the mating feature of the insertion object (moves with the robot)
feature_frame         = task_frame * pose_from_array(param.get("feature_frame"))
-- {h f_i}: origin of {f_i}, orientation of the housing
feature_frame_housing = frame(R_world_housing, origin(feature_frame))

-- ========================================= PARAMETERS ===================================
target_wrench    = param.get("target_wrench")
force_controlled = param.get("force_controlled")
reduced_dof      = param.get("reduced_dof")
K_t              = param.get("K_t")
K_r              = param.get("K_r")
K_force          = param.get("K_force")
maxvel           = param.get("maxvel")
maxvel_rot       = param.get("maxvel_rot")
tool_weight      = constant(param.get("tool_weight"))
tool_COG         = param.get("tool_COG")

DOFS = {"x", "y", "z", "rx", "ry", "rz"}
reduced_index = nil
for i, d in ipairs(DOFS) do
    if d == reduced_dof then reduced_index = i end
end
if force_controlled[reduced_index] ~= 1 or target_wrench[reduced_index] == 0 then
    error("insertion_action: the reduced DoF ("..reduced_dof..") must be force controlled (force_controlled = 1) with a non-zero target in target_wrench")
end

-- ========================================= MEASURED WRENCH ===================================
wrench_input = ctx:createInputChannelWrench(param.get("wrench_input_channel"))
if param.get("wrench_convention") == "exerted_on_environment" then
    wrench_on_robot = -wrench_input
else
    wrench_on_robot = wrench_input
end

function dead_zone(signal, dead_val)
    return conditional(abs(signal)-dead_val, signal + conditional(signal, -dead_val, dead_val), constant(0))
end
local ft = param.get("force_threshold")
local tt = param.get("torque_threshold")
wrench_dead_zone = wrench(vector(dead_zone(coord_x(force(wrench_on_robot)), ft), dead_zone(coord_y(force(wrench_on_robot)), ft), dead_zone(coord_z(force(wrench_on_robot)), ft)),
                          vector(dead_zone(coord_x(torque(wrench_on_robot)), tt), dead_zone(coord_y(torque(wrench_on_robot)), tt), dead_zone(coord_z(torque(wrench_on_robot)), tt)))

-- gravity compensation (as in guarded_motion)
d_g = vector(0,0,-1)
FT_sensor_frame_to_cog = frame(vector(constant(tool_COG[1]), constant(tool_COG[2]), constant(tool_COG[3])))
virtual_wrench = wrench(d_g*tool_weight, cross(origin(FT_sensor_frame_to_cog), d_g*tool_weight))
wrench_cog_ftframe = transform(rotation(inv(FT_sensor_frame)), wrench(d_g*tool_weight, vector(0,0,0)))
wrench_FT_frame = ref_point(wrench_cog_ftframe, -origin(FT_sensor_frame_to_cog))
wrench_compensated = wrench_dead_zone - wrench_FT_frame - virtual_wrench

-- the wrench with the origin of {f_i} as reference point, expressed in a frame F (with that origin)
local function wrench_in(F)
    return ref_point(transform(rotation(inv(F)*FT_sensor_frame), wrench_compensated), -origin(inv(F)*FT_sensor_frame))
end
wrench_housing = wrench_in(feature_frame_housing)   -- forces along the axes of {h f_i}
wrench_feature = wrench_in(feature_frame)           -- torques around the axes of {f_i}

measured = {coord_x(force(wrench_housing)), coord_y(force(wrench_housing)), coord_z(force(wrench_housing)),
            coord_x(torque(wrench_feature)), coord_y(torque(wrench_feature)), coord_z(torque(wrench_feature))}

-- =============================== INSTANTANEOUS FRAMES ==============================
-- displacements since the current sample: translations along the axes of {h f_i}, rotations around the axes of {f_i}
inst_housing = inv(make_constant(feature_frame_housing)) * feature_frame_housing
inst_feature = inv(make_constant(feature_frame)) * feature_frame
displacement = {coord_x(origin(inst_housing)), coord_y(origin(inst_housing)), coord_z(origin(inst_housing)),
                coord_x(getRotVec(rotation(inst_feature))), coord_y(getRotVec(rotation(inst_feature))), coord_z(getRotVec(rotation(inst_feature)))}

-- =============================== CONSTRAINTS ==============================
for i, d in ipairs(DOFS) do
    local rotational = i > 3
    if force_controlled[i] == 1 then
        -- admittance (Eq. 1): velocity = -K_force/K*(target - measured)
        Constraint{
            context  = ctx,
            name     = "force_"..d,
            model    = -(rotational and K_r or K_t)*displacement[i],
            meas     = measured[i],
            target   = target_wrench[i],
            K        = constant(K_force),
            weight   = constant(1),
            priority = 2,
        };
        local vmax = rotational and maxvel_rot or maxvel
        Constraint{
            context      = ctx,
            name         = "max_velocity_"..d,
            expr         = displacement[i],
            target_upper = vmax*time,
            target_lower = -vmax*time,
            K            = 0,
            weight       = 1,
            priority     = 1
        };
    else
        -- zero velocity: the current offset is kept
        Constraint{
            context  = ctx,
            name     = "zero_velocity_"..d,
            expr     = displacement[i],
            target   = constant(0),
            K        = 0,
            weight   = constant(1),
            priority = 2,
        };
    end
end

-- =============================== MONITOR ==============================
-- contact transition: the force/torque of the reduced DoF reaches monitor_fraction times its target
time_start = conditional(time - constant(param.get("time_delay")), constant(1), constant(0))
Monitor{
    context    = ctx,
    name       = "contact_transition_"..reduced_dof,
    upper      = param.get("monitor_fraction"),
    actionname = "exit",
    expr       = time_start*measured[reduced_index]/constant(target_wrench[reduced_index])
}

-- =============================== OUTPUTS ==============================
ctx:setOutputExpression("time", time)
ctx:setOutputExpression("x_f", coord_x(origin(feature_frame)))
ctx:setOutputExpression("y_f", coord_y(origin(feature_frame)))
ctx:setOutputExpression("z_f", coord_z(origin(feature_frame)))
ctx:setOutputExpression("feature_frame", feature_frame)
ctx:setOutputExpression("feature_frame_housing", feature_frame_housing)
ctx:setOutputExpression("x_force",  measured[1])
ctx:setOutputExpression("y_force",  measured[2])
ctx:setOutputExpression("z_force",  measured[3])
ctx:setOutputExpression("x_torque", measured[4])
ctx:setOutputExpression("y_torque", measured[5])
ctx:setOutputExpression("z_torque", measured[6])
