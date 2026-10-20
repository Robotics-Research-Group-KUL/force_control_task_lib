---------------------------------------------------------------------------
-- Basic filtering Toolbox for eTaSL
--
--  E. Aertbelien, June 2020.
---------------------------------------------------------------------------
require("pretty")
p = Pretty:new()
--
--  ANALOG filter specification required
--  higher order coefficients first.
--
--  Implemented using the controller canonical form.
--
function generate_filter(ctx, name, nominator, denominator,inp)
    local D = #denominator
    local N = #nominator
    if (N > D) then
        error("length of nominator cannot be larger  than length of denominator")
    end
    if (N ==0) or (D==0) then
        error("length of nominator and denominator should be >0")
    end
    local d = denominator[1]
    for i=D,1,-1 do
        if i <= D-N then
            nominator[i] = 0
        else
            nominator[i] = nominator[i-D+N]/d
        end
    end
    for i=1,D do
        denominator[i] = denominator[i]/d
    end
    local varlst={}
    for k=1,D-1 do
        table.insert( varlst, Variable{context=ctx,name=name.."_"..tostring(k),vartype="feature"})
    end
    p(nominator)
    p(denominator)
    -- for k,v in pairs(denominator) do
    for i=1,D-1 do
        local e = -denominator[i+1]*varlst[1] + (nominator[i+1] - denominator[i+1]*nominator[1] )*inp
        if i~=D-1 then 
            e = e + varlst[i+1]
        end
        print(i,e)
        Constraint {
            context  = ctx,
            name     = name.."_eq_"..tostring(i),
            expr     = varlst[i] - make_constant(e)*time,
            K        = 0,
            priority = 1
        }
    end
    local y = varlst[1] + nominator[1]*inp
    return y
end

function generate_butter_low_pass(ctx, name, freq,inp, order)
    local order = order or 2
    if order==2 then 
        local b = { freq*freq}
        local a = { 1,  math.sqrt(2)*freq, freq*freq } 
        return generate_filter(ctx,name,b,a,inp)
    elseif order==3 then
        local b = { freq*freq*freq }
        local a = { 1,  2*freq, 2*freq*freq, freq*freq*freq } 
        return generate_filter(ctx,name,b,a,inp)
    elseif order==4 then
        local f1=-2*math.cos( 5/8*pi)
        local f2=-2*math.cos( 7/8*pi) 
        local b = { freq*freq*freq*freq }
        local a = { 1, (f1+f2)*freq, (2+f1*f2)*freq*freq, (f1+f2)*freq*freq*freq, freq*freq*freq*freq}
        return generate_filter(ctx,name,b,a,inp)
    else
        error("can only handle order==2,3 or 4")
    end
end


function generate_butter_high_pass(ctx, name, freq,inp, order)
    local order = order or 2
    if order==2 then 
        local b = { 1, 0, 0}
        local a = { 1,  math.sqrt(2)*freq, freq*freq } 
        return generate_filter(ctx,name,b,a,inp)
    elseif order==3 then
        local b = { 1, 0, 0, 0 }
        local a = { 1,  2*freq, 2*freq*freq, freq*freq*freq } 
        return generate_filter(ctx,name,b,a, inp )
    elseif order==4 then
        local f1=-2*math.cos( 5/8*pi)
        local f2=-2*math.cos( 7/8*pi) 
        local b = { 1,0,0,0,0 }
        local a = { 1, (f1+f2)*freq, (2+f1*f2)*freq*freq, (f1+f2)*freq*freq*freq, freq*freq*freq*freq}
        return generate_filter(ctx,name,b,a, inp )
    else
        error("can only handle order==2,3 or 4")
    end

end



--[[
require("context")
inp = sin(time)
result = generate_filter(ctx, "my_filter",{0.25}, {1,0.70711 ,0.25000},inp)
--]]
