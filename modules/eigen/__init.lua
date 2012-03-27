local M = {}

-------------------------------------------------------------------------------
-- Types constructors
-------------------------------------------------------------------------------
function M.Vec3( x, y, z )
	out = co.new "eigen.Vec3"
	out:setXYZ( 0, 0, 0 )
	if z then
		out:setXYZ( x, y, z )
	end
	return out
end

function M.Mat4()
	out = co.new "eigen.Mat4"
	out:identity()
	return out
end

function  M.Quat( other )
	out = co.new "eigen.Quat"
	out:setWXYZ( 1, 0, 0, 0 )
	return out
end

--[[---------------------------------------------------------------------------
	All the functions mimics GLSL API. Except for an optional "out" parameter 
	that be used as	return value for efficiency. All functions create and 
	return a new type if "out" is nil. Do not pass the same reference to a 
	function's second parameter and out. 
		eg: 
			local v = eigen.vec3(); eigen.cross( otherV, v, v )
--]]---------------------------------------------------------------------------

--[[
	transforms given coordinates to [-1,1] (OpenGL like) space within given interval
	returns two values: normalized value of x and of y (assumes both screen and clip space
	to have y upwards)
--]]
function M.screenToClip( x, y, width, height )
	local cw = math.floor( width * 0.5 )
	local ch = math.floor( height * 0.5 )	
	return ( x / cw ) - 1.0, ( y / ch ) - 1.0
end

-- Constants
M.PI = 3.14159265358979323846
M.PI_2 = M.PI * 0.5
M.rad2deg = 180 / M.PI
M.deg2rad = M.PI / 180

-------------------------------------------------------------------------------
-- Vec3 functions
-------------------------------------------------------------------------------
-- returns the Vec3 cross product of Vec3 v1 against v2
function M.crossVec( v1, v2, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v1 )
	out:cross( v2 )
	return out	
end
	
-- returns the float dot product of Vec3 v1 and v2  
function M.dotVec( v1, v2 )
	return v1:dot( v2 )
end

-- returns the length o vector v
function M.length( v )
	return v:length()
end

-- returns the Vec3 linear interpolation from v1 to v2 at position 'factor'
function M.lerp( v1, v2, factor, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v1 )
	out:mix( v2, factor )
	return out	
end

-- returns the Vec3 normalized copy Vec3 v
function M.normalize( v, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:normalize()
	return out	
end

--------- Multiple Getters/Setters ---------

-- returns the float components of Vec3 v 
function M.getXYZ( v )
	return v:getXYZ()
end

-- sets the float components x,y,z of Vec3 v  
function M.setXYZ( v, x, y, z )
	v:setXYZ( x, y, z )
end

-------------------------------------------------------------------------------
-- Basic Operations
-------------------------------------------------------------------------------
-- returns the Vec3 result of the Vec3 v1 + Vec3 v2
function M.addVec( v1, v2, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v1 )
	out:add( v2 )
	return out	
end

-- returns the Vec3 result of v1 - v2
function M.subVec( v1, v2, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v1 )
	out:sub( v2 )
	return out	
end

-- returns the Vec3 result of Vec3 v * Scalar s
function M.mulVecScalar( v, s, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:mul( s )
	return out	
end

-- returns the Vec3 result of Vec3 v / Scalar s
function M.divVecScalar( v, s, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:mul( 1 / s )
	return out	
end

-- returns the Vec3 result of (inverse of Quat q) * Vec3 v
function M.mulVecQuat( v, q, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:mulVecQuat( q )
	return out	
end

-- returns the Vec3 result of Quat q * Vec3 v
function M.mulQuatVec( q, v, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:mulQuatVec( q )
	return out	
end

-------------------------------------------------------------------------------
-- Mat4 functions 
-------------------------------------------------------------------------------
-- returns m[i][j]
function M.getElement( m, i, j )
	return m:getElement( i, j )
end

-- returns the Mat4 containing the rotation of Quat q
function M.fromQuat( q, out )
	out = out or co.new "eigen.Mat4"
	out:fromQuat( q )
	return out
end

-- basic algebric operations returns the identity matrix
function M.identity( out ) 
	out = out or co.new "eigen.Mat4"
	out:identity()
	return out
end
	
-- returns the Mat4 inserve of m
function M.inverseMat( m, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:invert()
	return out	
end

-- returns the Mat4 inserve of m
function M.transpose( m, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:transpose()
	return out	
end

-- rotates a copy of Mat4 m by Scalar degrees around Vec3 axis and returns it
function M.rotate( m, degrees, axis, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:rotate( degrees, axis )
	return out	
end

-- returns a Mat4 that rotates a direction from Vec3 from to Vec3 to
function M.rotationFromToMat4( from, to, out )
	out = out or co.new "eigen.Mat4"
	out:rotationFromTo( from, to )
	return out	
end

-- scales a copy of m by the amount on each axis of Vec3 scale and returns it
function M.scale( m, scale, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:scale( scale )
	return out	
end

-- translates a copy of m by Vec3 position and returns it
function M.translate( m, position, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:translate( position )
	return out	
end

-- returns the Vec3 result of ( Mat4 m * Vec3 v )
function M.transform( m, v, out )
	out = out or co.new "eigen.Vec3"
	out:copy( v )
	out:transform( m )
	return out	 
end

-- returns the mat4 result of mat4 m1 + m2
function M.addMat( m1, m2, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m1 )
	out:add( m2 )
	return out
end

-- returns the mat4 result of mat4 m1 * m2
function M.mulMat( m1, m2, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m1 )
	out:preMul( m2 )
	return out
end

-- returns the mat4 result of mat4 m * double s
function M.mulMatScalar( m, s, out )
	out = out or co.new "eigen.Mat4"
	out:copy( m )
	out:mulScalar( s )
	return out
end

-------------------------------------------------------------------------------
-- Camera Operations functions 
-------------------------------------------------------------------------------
-- parameters: vec3 eye, vec3 center and vec3 up
function M.lookAt( eye, center, up, out )
	out = out or co.new "eigen.Mat4"
	out:lookAt( eye, center, up )
	return out
end

-- parameters: double left, right, bottom, top
function M.ortho( left, right, bottom, top, out )
	out = out or co.new "eigen.Mat4"
	out:ortho( left, right, bottom, top )
	return out
end

-- parameters: double fovy, aspect, zNear, zFar
function M.perspective( fovy, aspect, zNear, zFar, out )
	out = out or co.new "eigen.Mat4"
	out:perspective( fovy, aspect, zNear, zFar )
	return out
end

-- parameters: double left, right, bottom, top, nearVal, farVal
function M.frustum( left, right, bottom, top, nearVal, farVal, out )
	out = out or co.new "eigen.Mat4"
	out:frustum( left, right, bottom, top, nearVal, farVal )
	return out
end

-------------------------------------------------------------------------------
-- Quat functions 
-------------------------------------------------------------------------------
-- gets Quat q Scalar w,x,y,z coordinates
function M.getWXYZ( q )
	return q:getWXYZ()
end

-- sets Quat q Scalar w,x,y,z coordinates
function M.setWXYZ( q, w, x, y, z )
	q:setWXYZ( w, x, y, z )
end

-- returns the Scalar degrees Scalar w,x,y,z coordinates
function M.getAngleAxis( q )
	return q:getAngleAxis()
end

-- returns a Quat containing the rotation of Mat4 m
function M.fromMat4( m, out )
	out = out or co.new "eigen.Quat"
	out:fromMat4( m )
	return out
end

-- returns the Quat conjugate of q
function M.conjugate( q, out )
	out = out or co.new "eigen.Quat"
	out:copy( q )
	out:conjugate()
	return out
end

-- returns the Scalar dot product between Quat q1 and Quat q2
function M.dotQuat( q1, q2 )
	return q1:dot( q2 )
end

-- returns the Quat result of Quat q1 cross against Quat q2
function M.crossQuat( q1, q2, out )
	out = out or co.new "eigen.Quat"
	out:copy( q1 )
	out:cross( q2 )
	return out
end

-- returns the Quat inverse of Quat q
function M.inverseQuat( q, out )
	out = out or co.new "eigen.Quat"
	out:copy( q )
	out:inverse()
	return out
end

-- returns the Quat result of Quat q1 * Quat q2
function M.mulQuat( q1, q2, out )
	out = out or co.new "eigen.Quat"
	out:copy( q1 )
	out:mul( q2 )
	return out
end

-- returns the Quat result of shortest path SLERP between Quat q1 and q2 
function M.mix( q1, q2, factor, out )
	out = out or co.new "eigen.Quat"
	out:copy( q1 )
	out:mix( q2, factor )
	return out
end

-- rotates a copy of Quat q by Scalar degrees around Vec3 axis and returns it
function M.rotateQuat( q, degrees, axis, out )
	out = out or co.new "eigen.Quat"
	out:copy( q )
	out:rotate( degrees, axis )
	return out
end

-- returns a Quat that rotates a direction from Vec3 from to Vec3 to
function M.rotationFromToQuat( from, to, out )
	out = out or co.new "eigen.Quat"
	out:rotationFromTo( from, to )
	return out	
end

-------------------------------------------------------------------------------
-- Specific operators 
-------------------------------------------------------------------------------
local coTypeOf = co.typeOf

-- function for overloading the * operator for vec3 (tests which parameter is a scalar)
function M.vecMulOperator( a, b )
	typeA = coTypeOf( a )
	typeB = coTypeOf( b )
	if not typeA then
		return M.mulVecScalar( b, a )
	elseif not typeB then
		return M.mulVecScalar( a, b )
	elseif typeB == "eigen.Quat" then
		return M.mulVecQuat( a, b )
	else
		error( "there is no operation between a eigen.vec3 and the type passed" )
	end	
end

-- function for overloading the * operator for quat (tests which parameter is a scalar)
function M.quatMulOperator( a, b )
	typeB = coTypeOf( b )
	if typeB == "eigen.Quat" then
		return M.mulQuat( a, b )
	elseif typeB == "eigen.Vec3" then
		return M.mulQuatVec( a, b )
	else
		error( "there is no operation between a eigen.vec3 and the type passed" )
	end	
end

-- function for overloading the * operator for vec3 (tests which parameter is a scalar)
function M.vecDivOperator( a, b )
	if type( b ) == "number" then
		return M.divVecScalar( a, b )
	end
	return M.divVecScalar( b, a )
end

-- operator needs to check combinations of vec3, mat4 and scalar
function M.matMulOperator( a, b )
	typeA = coTypeOf( a )
	typeB = coTypeOf( b )
	if not typeA then
		return M.mulScalar( b, a )
	elseif typeB == "eigen.Mat4" then
		return M.mulMat( a, b )
	elseif not typeB then
		return M.mulMatScalar( a, b )
	elseif typeB == "eigen.Vec3" then
		return M.transform( a, b )
	else
		error( "there is no operation between a eigen.mat4 and the type passed" )
	end	
end

--operators for quat
M.zeroQuat = co.new "eigen.Quat"
local quatMT = getmetatable( M.zeroQuat )
quatMT.__mul = M.quatMulOperator

-- operators for mat4
M.idMat = co.new "eigen.Mat4"
local matMT = getmetatable( M.idMat )
matMT.__mul = M.matMulOperator
matMT.__add = M.addMat

-- operators for vec3
M.zeroVec = co.new "eigen.Vec3"
local vecMT = getmetatable( M.zeroVec )
vecMT.__add = M.addVec
vecMT.__sub = M.subVec
vecMT.__div = M.vecDivOperator
vecMT.__mul = M.vecMulOperator

return M
