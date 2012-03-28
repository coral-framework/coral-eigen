-----------------------------
-- Setup
-----------------------------

local eigen = require "eigen"

local TOLERANCE = 1e-7

local function ASSERT_VEC_EQ( v1, v2 )
	local x1,y1,z1 = eigen.getXYZ( v1 )
	local x2,y2,z2 = eigen.getXYZ( v2 )
	ASSERT_NEAR( x1, x2, TOLERANCE )
	ASSERT_NEAR( y1, y2, TOLERANCE )
	ASSERT_NEAR( z1, z2, TOLERANCE )
end

-----------------------------
-- Vec3 Tests
-----------------------------

function basicsTest()
	local v1 = eigen.Vec3( 1, 1, 1 )
	local v2 = eigen.Vec3()
	v2.x = 1
	v2.y = 1
	v2.z = 1
	ASSERT_VEC_EQ( v1, v2 )
end

function mtOperationsTest()
	local v1 = eigen.Vec3( 1, 1, 1 )
	local v2 = eigen.Vec3( 1, 1, 1 )
	local v3 = v1 + v2
	ASSERT_VEC_EQ( v3, eigen.Vec3( 2, 2, 2 ) )
	v1 = v2 + v3
	ASSERT_VEC_EQ( v1, eigen.Vec3( 3, 3, 3 ) )

	v1 = eigen.Vec3()
	v2 = eigen.Vec3( 1, 1, 1 )
	v3 = v2 * 2
	ASSERT_VEC_EQ( v3, eigen.Vec3( 2, 2, 2 ) )
	v1 =  v2 * 2
	ASSERT_VEC_EQ( v1, v3 )
end

function basicOpsTest()
	local v1 = eigen.Vec3( 1, 1, 1 )
	local v2 = eigen.Vec3( 1, 1, 1 )
	local v3 = eigen.Vec3()
	eigen.addVec( v1, v2, v3 )
	ASSERT_VEC_EQ( v3, eigen.Vec3( 2, 2, 2 ) )
	v1 = eigen.addVec( v2, v3 )
	ASSERT_VEC_EQ( v1, eigen.Vec3( 3, 3, 3 ) )

	v1 = eigen.Vec3( 1, 1, 1 )
	v2 = eigen.Vec3( 1, 1, 1 )
	v3 = eigen.Vec3()
	eigen.subVec( v1, v2, v3 )
	ASSERT_VEC_EQ( v3, eigen.Vec3( 0, 0, 0 ) )
	v1 = eigen.addVec( v2, v3 )
	ASSERT_VEC_EQ( v1, eigen.Vec3( 1, 1, 1 ) )

	v1 = eigen.Vec3()
	v2 = eigen.Vec3( 1, 1, 1 )
	v3 = eigen.Vec3()
	eigen.mulVecScalar( v2, 2, v3 )
	ASSERT_VEC_EQ( v3, eigen.Vec3( 2, 2, 2 ) )
	v1 = eigen.mulVecScalar( v2, 2 )
	ASSERT_VEC_EQ( v1, v3 )
end

function crossTest()
	local v1 = eigen.Vec3( 0, 1, 0 )
	local v2 = eigen.Vec3( 0, 0, 1 )
	local v3 = eigen.crossVec( v1, v2 )
	ASSERT_VEC_EQ( v3, eigen.Vec3( 1, 0, 0 ) )
end

function dotTest()
	local v1 = eigen.Vec3( 0, 1, 0 )
	local v2 = eigen.Vec3( 0, 0, 1 )
	local result = eigen.dotVec( v1, v2 )
	ASSERT_DOUBLE_EQ( result, 0 )
	v1.z = 1
	result = eigen.dotVec( v1, v2 )
	eigen.crossVec( v1, v2, v4 )
	ASSERT_DOUBLE_EQ( result, 1 )
end

function normLengthTest()
	local v1 = eigen.Vec3( 0, 3, 4 )
	local v2 = eigen.Vec3()
	ASSERT_DOUBLE_EQ( eigen.length( v1 ), 5 )
	eigen.normalize( v1, v2 )
	ASSERT_VEC_EQ( v2, eigen.Vec3( 0, 3/5, 4/5 ) )
end

function QuatMulTest()
	local v1 = eigen.Vec3( 1, 0, 0 )
	local q = eigen.Quat()
	eigen.rotateQuat( q, 90, eigen.Vec3( 0, 1, 0 ), q )
	local v2 = eigen.Vec3( 0, 0, -1 )
	v1 = q * v1
	ASSERT_VEC_EQ( v1, v2 )
	v1 = eigen.Vec3( 1, 0, 0 )
	v2 = eigen.Vec3( 0, 0, 1 )
	v1 = v1 * q
	ASSERT_VEC_EQ( v1, v2 )
end
