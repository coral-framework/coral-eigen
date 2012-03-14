-----------------------------
-- Setup
-----------------------------

local eigen = require "eigen"

local DOUBLE_TOLERANCE = 1e-7

-----------------------------
-- Quaternion Tests
-----------------------------

local function ASSERT_QUAT_EQ( q1, q2 )
	local w1,x1,y1,z1 = eigen.getWXYZ( q1 )
	local w2,x2,y2,z2 = eigen.getWXYZ( q2 )
	ASSERT_NEAR( w1, w2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( x1, x2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( y1, y2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( z1, z2, DOUBLE_TOLERANCE )
end

local q1
local q2
local q3
local q4
local m1
local v1

function invMulTest()
	q1 = eigen.Quat()
	q2 = eigen.Quat()
	q3 = eigen.Quat()
	eigen.setWXYZ( q1, 0, 1, 1, -1 )	
	eigen.inverseQuat( q1, q2 )
	eigen.mulQuat( q1, q2, q3 )
	q1 = eigen.Quat()
	ASSERT_QUAT_EQ( q1, q3 )
end

function operatorTest()
	q1 = eigen.Quat()
	q2 = eigen.Quat()
	q3 = eigen.Quat()
	eigen.setWXYZ( q1, 0, 1, 1, -1 )
	eigen.inverseQuat( q1, q2 )
	q3 = q1 * q2
	q1 = eigen.Quat()
	ASSERT_QUAT_EQ( q1, q3 )
end

function mixRotTest()
	q1 = eigen.Quat()
	q2 = eigen.Quat()
	eigen.setWXYZ( q1, 0, 0, 1, 0 )
	eigen.setWXYZ( q2, 1, 1, -1, 0 )
	eigen.mix( q1, q2, 1/2, q1 )
	eigen.setWXYZ( q2, -1/2, -1/2, 1, 0 )
	ASSERT_QUAT_EQ( q1, q2 )
	eigen.rotateQuat( q1, 180, eigen.Vec3( 0, 1, 0 ), q1 )
	eigen.setWXYZ( q2, -1, 0, -1/2, -1/2 )
	ASSERT_QUAT_EQ( q1, q2 )
end

function rotationFromToTest()
	local v1 = eigen.Vec3( 1, -1, 1 )
	local v2 = eigen.Vec3( 10, 0.001, 0.0 )
	local q = eigen.Quat()
	local qConjugate = eigen.Quat()
	eigen.rotationFromToQuat( v1, v2, q )
	eigen.rotationFromToQuat( v2, v1, qConjugate )
	eigen.normalize( q * v1, v1 )
	eigen.normalize( v2, v2 )
	local x1,y1,z1 = eigen.getXYZ( v1 )
	local x2,y2,z2 = eigen.getXYZ( v2 )
	ASSERT_NEAR( x1, x2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( y1, y2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( z1, z2, DOUBLE_TOLERANCE )
	ASSERT_QUAT_EQ( qConjugate, eigen.conjugate( q ) ) 
end

function getAngleAxisTest()
	local q = eigen.Quat()
	eigen.setWXYZ( q, 0, 0, 1, 0 )
	eigen.rotateQuat( q, 180, eigen.Vec3( 0, 1, 0 ) )
	
	local angle, axis = eigen.getAngleAxis( q )
	ASSERT_NEAR( angle, 180 )
	ASSERT_NEAR( axis.x, 0 )
	ASSERT_NEAR( axis.y, 1 )
	ASSERT_NEAR( axis.z, 0 )
end

function getAngleAxisTest2()
	local q = eigen.Quat()
	eigen.setWXYZ( q, 0, 0, 1, 0 )
	eigen.rotateQuat( q, 180, eigen.Vec3( 1, 0, 0 ) )
	
	local angle, axis = eigen.getAngleAxis( q )
	ASSERT_DOUBLE_EQ( angle, 180, DOUBLE_TOLERANCE )
	ASSERT_DOUBLE_EQ( axis.x, 0, DOUBLE_TOLERANCE )
	ASSERT_DOUBLE_EQ( axis.y, 1, DOUBLE_TOLERANCE )
	ASSERT_DOUBLE_EQ( axis.z, 0, DOUBLE_TOLERANCE )
end

function dotCrossConjTest()
	q1 = eigen.Quat()
	q2 = eigen.Quat()
	eigen.setWXYZ( q1, 0, 0, 1, 0 )
	eigen.setWXYZ( q2, 0, 0, -1, 0 )
	eigen.conjugate( q1, q1 )
	ASSERT_QUAT_EQ( q1, q2 )
	eigen.setWXYZ( q2, 1, 1, -1, 0 )
	eigen.setWXYZ( q1, 0, 0, 1, 0 )
	local result = eigen.dotQuat( q1, q2 )
	q1 = eigen.crossQuat( q1, q2 )
	eigen.setWXYZ( q2, 1, 0, 1, -1 )
	ASSERT_DOUBLE_EQ( result, -1 )
	ASSERT_QUAT_EQ( q1, q2 )
end

--[[
function getSetMatTest()
	q1 = eigen.Quat()
	q2 = eigen.Quat()
	eigen.setWXYZ( q2, 0, 0, 1, 0 )
	m1 = eigen.Mat4()
	eigen.rotate( m1, 180, eigen.Vec3( 0, 1, 0 ), m1 )
	eigen.fromMat4( m1, q1 )
	ASSERT_QUAT_EQ( q1, q2 )
end
]]--
