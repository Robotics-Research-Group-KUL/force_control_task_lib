--------------------------------------------------------------------------
--
-- eTaSL specification of everything related to our robot/set-up 
-- configuration in pyBullet.
--
-- E. Aertbelien, May 2020
--------------------------------------------------------------------------
require("context")
require("pretty")
require("geometric")
require("taskframe_formalism")

--require("filters")

p = Pretty:new()  -- p(..) is used for printing tables etc...

--------------------------------------------------------------
-- read robot model and robot-related constraints:
-- define two expressions: for the end effector wrt to the root
-- and for the elbow wrt to the root
-- use robot:writeDot("kuka_iiwa.dot") to generate a graph
--------------------------------------------------------------
worldmodel=require("worldmodel")
urdfreader=require("urdfreader")


xmlstr = urdfreader.loadFile("data/my_kuka.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
VL = {}
rv = robot:getExpressions(VL,ctx,
{
        ee={'tooltip','root'},
        elbow={'lbr_iiwa_link_4','root'}
})
ee = rv['ee']
elbow = rv['elbow']

robot_jpos = {
    VL.lbr_iiwa_joint_1,
    VL.lbr_iiwa_joint_2,
    VL.lbr_iiwa_joint_3,
    VL.lbr_iiwa_joint_4,
    VL.lbr_iiwa_joint_5,
    VL.lbr_iiwa_joint_6,
    VL.lbr_iiwa_joint_7,
}
--robot:writeDot("kuka_iiwa.dot")
--p(VL)

--------------------------------------------------------------
-- A starting position for our application:
--------------------------------------------------------------
tgt_x = constant(0.76)
tgt_y = constant(0.0)
tgt_z = constant(0.08)
-- target pose:  (with z-axis pointing down)
tgt = frame(rot_x(constant(pi)),vector(tgt_x,tgt_y,tgt_z))

--------------------------------------------------------------
-- FORCE INPUT SECTION
--  (and possibly other input signals)
--------------------------------------------------------------
--
Fx    = -ctx:createInputChannelScalar("Fx")
Fy    = -ctx:createInputChannelScalar("Fy")
Fz    = -ctx:createInputChannelScalar("Fz")
Tx    = -ctx:createInputChannelScalar("Tx")
Ty    = -ctx:createInputChannelScalar("Ty")
Tz    = -ctx:createInputChannelScalar("Tz")

--[[
ffreq = 10 
order = 2

Fx = generate_butter_low_pass(ctx,'Fx',ffreq, Fx,order)
Fy = generate_butter_low_pass(ctx,'Fy',ffreq, Fy,order)
Fz = generate_butter_low_pass(ctx,'Fz',ffreq, Fz,order)
Tx = generate_butter_low_pass(ctx,'Tx',ffreq, Tx,order)
Ty = generate_butter_low_pass(ctx,'Ty',ffreq, Ty,order)
Tz = generate_butter_low_pass(ctx,'Tz',ffreq, Tz,order)
--]]




--------------------------------------------------------------
-- TASK 5 : Follow contour on forces
--
-- E. Aertbelien, May 2020
--------------------------------------------------------------
require("force_control_tutorial_robot")
require("taskframe_formalism")
require("geometric")
--[[
Constraint {
    context = ctx,
    name         = "j3",
    expr         = VL.lbr_iiwa_joint_3,
    target_lower = 0.9,
    target_upper = 1.5,
    priority     = 2,
    K            = 4,
    weight       = 0.01
}
--]]

fv = Variable{context=ctx,name="fv",vartype="feature"}
R  = rotate_z(fv) 

TF = TaskFrame:new {
    context    = ctx,
    taskframe  = ee*R,
    K          = 3,   -- 3
    K_adapt    = 0.9, -- 0.7
    Ke         = 5000,-- 3000
    meas       = wrench(vector(Fx,Fy,Fz),vector(Tx,Ty,Tz)),
    ft_frame   = ee,
    ft_cog     = origin(ee),
    ft_mass    = 0.0,
    gravity    = Vector(0,0,-9.81),
    priority   = 1
}

TF:setForce(                X,  30            )
TF:setVelocity(             Y,  0.02          )
TF:setVelocity(             Z,  0             )
TF:setRotationalVelocity(   X,  0             )
TF:setRotationalVelocity(   Y,  0             )
TF:trackOrientationOnForce( Z,  Vector(1,0,0) )


Constraint{
    context=ctx,
    expr=coord_z(origin(ee)),
    target=tgt_z,
    K=4
}
    

TF2 = TaskFrame:new {
    context    = ctx,
    taskframe  = ee,
    K          = 4,   -- 3
    priority   = 1
}
TF2:setRotationalVelocity(Z, 0, {priority=2})

PL = TF:createPathlength()
Monitor {
    context = ctx,
    name    = "pathlength",
    expr    = PL,
    upper   = 1.0, 
    actionname = "exit"
}
wr = TF:externalWrench()

roll,pitch,yaw = getRPY(rotation(ee))
ctx:setOutputExpression("x", coord_x(origin(ee))) 
ctx:setOutputExpression("y", coord_y(origin(ee))) 
ctx:setOutputExpression("z", coord_z(origin(ee))) 
ctx:setOutputExpression("yaw", yaw) 
ctx:setOutputExpression("pitch", pitch) 
ctx:setOutputExpression("roll", roll) 
ctx:setOutputExpression("tan_vel", TF:magnitudeVelocity() )
ctx:setOutputExpression("pathlength", PL) 
ctx:setOutputExpression("external Fx", coord_x(force(wr)) )
ctx:setOutputExpression("external Fy", coord_y(force(wr)) )
ctx:setOutputExpression("external Fz", coord_z(force(wr)) )
ctx:setOutputExpression("external Tx", coord_x(torque(wr)) )
ctx:setOutputExpression("external Ty", coord_y(torque(wr)) )
ctx:setOutputExpression("external Tz", coord_z(torque(wr)) )


print(ctx)
