-----------------------------
-- Setup
-----------------------------

local eigen = require "eigen"

-----------------------------
-- Matrix Tests
-----------------------------

local DOUBLE_TOLERANCE = 1e-7

local m1
local m2
local m3
local m4

function operatorsTest()
	local v1 = eigen.Vec3( 0, 0, 0 )
	m1 = eigen.Mat4()
	
	v1 = m1 * v1
	ASSERT_DOUBLE_EQ( v1.x, 0 )
	ASSERT_DOUBLE_EQ( v1.y, 0 )
	ASSERT_DOUBLE_EQ( v1.z, 0 )
	
	m1 = eigen.translate( m1, eigen.Vec3( 2, 2, 2 ) )
	v1 = m1 * v1
	ASSERT_DOUBLE_EQ( v1.x, 2 )
	ASSERT_DOUBLE_EQ( v1.y, 2 )
	ASSERT_DOUBLE_EQ( v1.z, 2 )
	
	m2 = eigen.inverseMat( m1 )
	m3 = m1 * m2
	local i
	local j
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 1 or 0, DOUBLE_TOLERANCE )
		end
	end
	
	m3 = m3 + m3 
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 2 or 0, DOUBLE_TOLERANCE )
		end
	end
	
	m3 = m3 * 2
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 4 or 0, DOUBLE_TOLERANCE )
		end
	end
end

function elementIdRST()
	m1 = eigen.Mat4()
	m2 = eigen.Mat4()
	ASSERT_NEAR( eigen.getElement( m1, 0, 0 ), 1, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 1 ), 1, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 2 ), 0, DOUBLE_TOLERANCE )
	
	eigen.rotate( m1, 90, eigen.Vec3( 0, 1, 0 ),  m1, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 0 ), 0, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 0 ), -1, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 2 ), 1, DOUBLE_TOLERANCE )
	
	eigen.scale( m2, eigen.Vec3( 2, 2, 2 ), m2 )
	ASSERT_NEAR( eigen.getElement( m2, 0, 0 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 1, 1 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 2 ), 2, DOUBLE_TOLERANCE )
	
	eigen.identity( m1 )
	eigen.translate( m1, eigen.Vec3( 2, 2, 2 ), m1, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), 2, DOUBLE_TOLERANCE	)
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), 2, DOUBLE_TOLERANCE )
end

function invTranpTest()
	m1 = eigen.Mat4()
	
	eigen.translate( m1, eigen.Vec3( 2, 2, 2 ), m1 )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), 2, DOUBLE_TOLERANCE )
	
	eigen.inverseMat( m1, m1 )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), -2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), -2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), -2, DOUBLE_TOLERANCE )
	
	eigen.transpose( m1, m1 )
	ASSERT_NEAR( eigen.getElement( m1, 3, 0 ), -2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 3, 1 ), -2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 3, 2 ), -2, DOUBLE_TOLERANCE )
end

function rotationFromToTest()
	local v1 = eigen.Vec3( 1, 1, 1 )
	local v2 = eigen.Vec3( 0, 10, 23 )
	local m = eigen.Mat4()
	eigen.rotationFromToMat4( v1, v2, m )
	eigen.normalize( m * v1, v1 )
	eigen.normalize( v2, v2 )
	local x1,y1,z1 = eigen.getXYZ( v1 )
	local x2,y2,z2 = eigen.getXYZ( v2 )
	ASSERT_NEAR( x1, x2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( y1, y2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( z1, z2, DOUBLE_TOLERANCE )
end

function fromQuatTest()
	local q = eigen.Quat()
	eigen.setWXYZ( q, 0, 0, 1, 0 )
	m1 = eigen.Mat4()
	eigen.rotate( m1, 180, eigen.Vec3( 0, 1, 0 ), m1 )
	m2 = eigen.fromQuat( q )
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m1, i, j ), eigen.getElement( m2, i, j ), DOUBLE_TOLERANCE )
		end
	end
end

function cameraTest()
	m1 = eigen.lookAt( eigen.Vec3( 0, 0, -1 ), eigen.Vec3( 0, 0, 1 ), eigen.Vec3( 0, 1, 0 ) )
	ASSERT_NEAR( eigen.getElement( m1, 0, 0 ), -1 )
	ASSERT_NEAR( eigen.getElement( m1, 2, 2 ), -1 )
	--translation
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), 0 )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), 0 )
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), 1 )
	
	m2 = eigen.frustum( -1, 1, -1, 1, 2, 4 )
	ASSERT_NEAR( eigen.getElement( m2, 0, 0 ), 2, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 2 ), -3, DOUBLE_TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 3 ), -8, DOUBLE_TOLERANCE )
	
	m3 = eigen.ortho( -1, 1, -1, 1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 0, 0 ), 1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 2, 2 ), -1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 2, 3 ), 0 )
	
	m4 = eigen.perspective( 3.1415 / 2, 1.0, 1.0, 10.0 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 0, 0 ), 72.948834620885 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 2, 2 ), -11/9 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 3, 2 ), -1 )
	
	m1 = m3 * m1
	ASSERT_DOUBLE_EQ( eigen.getElement( m1, 0, 0 ), -1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m1, 2, 2 ), 1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m1, 2, 3 ), -1 )
end

