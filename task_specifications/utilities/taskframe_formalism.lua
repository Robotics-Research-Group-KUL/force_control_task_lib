-------------------------------------------------------------------------
-- Small framework for the task frame formalism
--
-- v2 with a different API:
--    a task frame data structure and functions that perform 
--    operations on it.  The advantage is that it easier to
--    handle default parameters and multiple parameters for
--    a force or velocity specification.
--
--    Work in progress
--
-- E. Aertbelien May 2020
-------------------------------------------------------------------------
require("context")
require("pretty")
require("velocities")
p = Pretty:new()  -- p(..) is used for printing tables etc...

TaskFrame = {
    objtype = "TaskFrame"
}

X=1
Y=2
Z=3

local function select_coord(dir,v)
    if dir==X then
        return coord_x(v)
    elseif dir==Y then
        return coord_y(v)
    elseif dir==Z then
        return coord_z(v)
    else
        error("dir argument not recognized",3)
    end
end

local function str_coord(dir)
    if dir==X then
        return "X"
    elseif dir==Y then
        return "Y"
    elseif dir==Z then
        return "Z"
    end
    return "?"
end



function TaskFrame:new(arg)
    if arg==nil then
        error("TaskFrame::new() requires table argument",2)
    end
    setmetatable(arg, self)
    self.__index  = self
    if arg.taskframe==nil then
        error("no taskframe specified",2)
    end
    if arg.context==nil then
        error("no context specified",2)
    end
    arg.name     = arg.name or "taskframe"
    arg.tfic     = cached(inv(make_constant(arg.taskframe)))
    if arg.meas~=nil then
        if arg.ft_frame==nil then
            error("if you specify force measurements, you have to specify a force/torque measurement frame (ft_frame)",2)
        end 
        if arg.ft_cog==nil then
            error("if you specify force measurements, you need to specify the center of gravity of the mass after the F/T sensor (ft_cog)",2)
        end
        if arg.ft_mass==nil then
            error("if you specify force measurements, you need to specify the mass after the F/T sensor (ft_cog)",2)
        end
        if arg.gravity==nil and arg.ft_mass~=0 then
            error("if you specify force measurements and a non-zero ft_mass, you need to specify the gravity vector (gravity)",2)
        end
        -- F/T meas translated to task frame:
        local ft_wrt_taskframe      = inv(arg.taskframe)*arg.ft_frame
        local relpos                = origin( ft_wrt_taskframe)
        local R                     = cached( rotation(ft_wrt_taskframe) )
        local measF_wrt_taskframe   = cached( R * force(arg.meas)        )
        local measT_wrt_taskframe   = cached( R * torque(arg.meas) + cross(relpos,measF_wrt_taskframe)  )

        -- gravity estimates translated to task frame:
        local cog_wrt_taskframe     = inv(arg.taskframe)*arg.ft_cog
        local R                     = cached(inv(rotation(arg.taskframe)))
        local gravF_wrt_taskframe   = cached(arg.ft_mass* (R * arg.gravity) )
        local gravT_wrt_taskframe   = cached(cross(cog_wrt_taskframe,gravF_wrt_taskframe)  ) 
        
        -- compensate gravity to obtain external forces acting ON the taskframe, wrt taskframe:
        arg.ft_meas = wrench( measF_wrt_taskframe-gravF_wrt_taskframe, measT_wrt_taskframe-gravT_wrt_taskframe)
        arg.meas=nil -- avoid bugs that use arg.meas
    end
    arg.K        = arg.K or 4.0
    arg.K_adapt  = arg.K_adapt or K
    arg.priority = arg.priority or 2
    arg.Ke       = arg.Ke
    arg.weight    = arg.weight or 1.0
    arg.rotweight = arg.rotweight or arg.weight
    return arg
end

-- INTERNAL ROUTINE:
-- gets and applies the default values for:
--     K, K_adapt, priority, Ke, weight, rotweight
-- if needed and passes  possibly additional parameters
-- and returns the resutlign table.
--
function TaskFrame:get_default_param(arg)
    if arg==nil then
        arg={}
    end
    rval=arg
    rval.K         = arg.K or self.K
    rval.K_adapt   = arg.K_adapt or self.K_adapt 
    rval.priority  = arg.priority or self.priority 
    rval.Ke        = arg.Ke or self.Ke
    rval.weight    = arg.weight or self.weight
    rval.rotweight = arg.rotweight or self.rotweight 
    return rval
end

-- sets the translational velocity in the direction dir to vel
-- relevant parameters for param are:
--  priority, weight, 
function TaskFrame:setVelocity(dir, vel, param)
    local p = self:get_default_param(param)
    ScalarVelocityConstraint {
        context         = self.context,
        name            = self.name.."_vel_"..str_coord(dir),
        expr            = select_coord(dir, self.tfic* origin(self.taskframe) ),
        target_velocity = vel,
        priority = p.priority,
        weight   = p.weight
    }
end 

--[[

These do not bring improvement to the performance:

function TaskFrame:keepPosition(dir, param)
    local p = self:get_default_param(param)
    local e    = origin(self.taskframe)
    local tgt  = initial_value(time,e)
 
    --ctx:setOutputExpression("keepPos-err",select_coord(dir,e-tgt))
    Constraint {
        context         = self.context,
        name            = self.name.."_pos_"..str_coord(dir),
        expr            = select_coord(dir,rotation(self.tfic)*e),
        target          = select_coord(dir,rotation(self.tfic)*tgt),
        K             = p.K,
        priority      = p.priority,
        weight        = p.weight
    }
end 


function TaskFrame:keepOrientation(dir, param)
    local p = self:get_default_param(param)
    local e    = rotation(self.taskframe)
    local tgt  = initial_value(time,e)
 
    --ctx:setOutputExpression("keepPos-err",select_coord(dir,e-tgt))
    Constraint {
        context         = self.context,
        name            = self.name.."_orient_"..str_coord(dir),
        expr            = select_coord(dir,getRotVec(inv(tgt)*e)),
        K             = p.K,
        priority      = p.priority,
        weight        = p.rotweight
    }
end 

--]]


-- sets the rotational velocity in the direction dir to vel
-- relevant parameters for param are:
--  priority, rotweight, 
function TaskFrame:setRotationalVelocity(dir, vel, param)
    local p = self:get_default_param(param)
    ScalarVelocityConstraint {
        context  = self.context,
        name     = self.name.."_rotvel_"..str_coord(dir),
        expr     = select_coord(dir, getRotVec( rotation(self.tfic*self.taskframe))),
        target_velocity = vel,
        priority = p.priority,
        weight   = p.rotweight
    }
end 

-- sets the desired force in the direction dir to Fdes
-- relevant parameters for param are:
--  Ke,K,priority, weight, 
function TaskFrame:setForce(dir, Fdes, param)
    local p = self:get_default_param(param)
    Constraint {
        context =  self.context,
        name    =  self.name.."_force_"..str_coord(dir),
        model   =  -p.Ke* select_coord(dir, self.tfic * origin(self.taskframe) ),
        meas    =  select_coord(dir,force(self.ft_meas)),
        target  =  Fdes,
        K       =  p.K,
        priority = p.priority,
        weight   = p.weight
    }
end


-- sets the desired torque in the direction dir to Tdes
-- relevant parameters for param are:
--  Ke,K,priority, rotweight,
--  (alternatively you can use weight) 
function TaskFrame:setTorque(dir, Tdes, param)
    param=param or {}
    if (param.weight~=nil) and (param.rotweight==nil) then
        param.rotweight = param.weight
    end
    local p = self:get_default_param(param)
    Constraint {
        context =  self.context,
        name    = self.name.."_torque_"..str_coord(dir),
        model   = - p.Ke*select_coord(dir,getRotVec( rotation(self.tfic*self.taskframe)) ),
        meas    =  select_coord(dir,force(self.ft_meas)),
        target  =  Tdes,
        K       =  p.K,
        priority = p.priority,
        weight   = p.rotweight
    }
end

-- changes the orientation such that the force is aligned
-- align by rotating around axis such that the force is aligned
-- to direction alignTo.  alignTo is a vector and the axis
-- is a task frame direction (X,Y,Z)
--
-- angle from F to alignTo along axis dir:
--      - project both in plane perpendicular to axis
--
-- relevant parameters are:
--    K_adapt
--
-- For the numerically stable way of computing the angle between two vectors, see:
--   atan2 ( d x F,  d . F ) is used. (angle from d to F), projected in the plane perpendicular to the axis.
--
function TaskFrame:trackOrientationOnForce(axis,alignTo,param)
    param=param or {}
    if (param.weight~=nil) and (param.rotweight==nil) then
        param.rotweight = param.weight
    end
    local p = self:get_default_param(param)
    local d1,d,F1,F2
    alignTo = convert_to_expression(alignTo)
    if axis==Z then      -- 1=X, 2=Y
       d1 = coord_x(alignTo) 
       d2 = coord_y(alignTo) 
       F1 = coord_x( force(self.ft_meas))
       F2 = coord_y( force(self.ft_meas))
    elseif axis==X then -- 1=Y 2=Z
       d1 = coord_y(alignTo) 
       d2 = coord_z(alignTo) 
       F1 = coord_y( force(self.ft_meas))
       F2 = coord_z( force(self.ft_meas))
    elseif axis==Y then -- 1=Z 2=X
       d1 = coord_z(alignTo) 
       d2 = coord_x(alignTo) 
       F1 = coord_z( force(self.ft_meas))
       F2 = coord_x( force(self.ft_meas))
    end
        self.context:setOutputExpression("angular_vel_tracking", p.K_adapt*atan2( d1*F2-d2*F1, F1*d1+F2*d2 ))
        ScalarVelocityConstraint {
            context         = self.context,
            name            = self.name.."_trackOnForce_"..str_coord(axis),
            expr            = select_coord(axis, getRotVec( rotation(self.tfic*self.taskframe))),
            target_velocity = p.K_adapt * atan2( d1*F2-d2*F1, F1*d1+F2*d2 ),
            priority = p.priority,
            weight   = p.rotweight
        }
end


-- changes the orientation such that the velocity is aligned
-- align by rotating around axis such that the velocity is aligned
-- to direction alignTo.  alignTo is a vector and the axis
-- is a task frame direction (X,Y,Z)
--
-- angle from velocity to alignTo along axis dir:
--      - project both in plane perpendicular to axis
--
-- relevant parameters are:
--    K_adapt
function TaskFrame:trackOrientationOnVelocity(axis,alignTo,param)
    param=param or {}
    if (param.weight~=nil) and (param.rotweight==nil) then
        param.rotweight = param.weight
    end
    local p = self:get_default_param(param)
    local d1,d,V1,V2
    alignTo = convert_to_expression(alignTo)
    vel     = previous_velocity(time, origin(self.taskframe))
    vel  = conditional( norm(vel)-5E-3, vel,alignTo )
    if axis==Z then      -- 1=X, 2=Y
       d1 = coord_x(alignTo) 
       d2 = coord_y(alignTo) 
       V1 = coord_x( vel )
       V2 = coord_y( vel )
    elseif axis==X then -- 1=Y 2=Z
       d1 = coord_y(alignTo) 
       d2 = coord_z(alignTo) 
       V1 = coord_y( vel )
       V2 = coord_z( vel )
    elseif axis==Y then -- 1=Z 2=X
       d1 = coord_z(alignTo) 
       d2 = coord_x(alignTo) 
       V1 = coord_z( vel )
       V2 = coord_x( vel )
    end
    --if p.moving==nil then
        self.context:setOutputExpression("atan2", atan2( d1*V2-d2*V1, V1*d1+V2*d2 ))
        self.context:setOutputExpression("d1",  d1)
        self.context:setOutputExpression("d2",  d2)
        self.context:setOutputExpression("V1",  V1)
        self.context:setOutputExpression("V2",  V2)

        ScalarVelocityConstraint {
            context         = self.context,
            name            = self.name.."_trackOnVelocity_"..str_coord(axis),
            expr            = select_coord(axis, getRotVec( rotation(self.tfic*self.taskframe))),
            target_velocity = p.K_adapt * atan2( d1*V2-d2*V1, V1*d1+V2*d2 ),
            priority = p.priority,
            weight   = p.rotweight
        }
    --else
    --    local tf = make_constant(self.ft_frame)*inv(self.ft_frame)*self.taskframe
    --    ScalarVelocityConstraint {
    --        context         = self.context,
    --        name            = self.name.."_trackOnVelocityMoving_"..str_coord(axis),
    --        expr            = select_coord(axis, getRotVec( rotation(self.tfic*tf))),
    --        target_velocity = p.K_adapt * atan2( d1*V2-d2*V1, V1*d1+V2*d2 ),
    --        priority = p.priority,
    --        weight   = p.rotweight
    --    }
    --end
end



--
-- Gives back an expression for the path length that the
-- origin of a task frame travels.
-- CAVEAT: (1) each call to this routine creates a feature variable.
--         It it better to reuse the expression instead of recalling 
--         this routine.
function TaskFrame:createPathlength()
    local pathlength = Variable{ context=self.context, name=self.name.."_pathlength", vartype='feature', initial=0.0 }
    ScalarVelocityConstraint {
            context         = self.context,
            name            = self.name.."_pathlength",
            expr            = pathlength,
            target_velocity = norm(previous_velocity(time, origin(self.taskframe))),
            priority        = 1 
    }
    return pathlength
end

--
-- gets an expression for the external wrench expressed 
-- in the task frame.
--
function TaskFrame:externalWrench()
    return self.ft_meas
end

--
-- gets the magnitude of the velocity of the origin of the 
-- taskframe.
--
function TaskFrame:magnitudeVelocity()
    return norm(previous_velocity(time, origin(self.taskframe)))
end

