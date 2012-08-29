-----------------------------
-- Setup
-----------------------------

local eigen = require "eigen"

-----------------------------
-- Matrix Tests
-----------------------------

local TOLERANCE = 1e-7

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
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 1 or 0, TOLERANCE )
		end
	end

	m3 = m3 + m3
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 2 or 0, TOLERANCE )
		end
	end

	m3 = m3 * 2
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m3, i, j ), i == j and 4 or 0, TOLERANCE )
		end
	end
end

function elementIdRST()
	m1 = eigen.Mat4()
	m2 = eigen.Mat4()
	ASSERT_NEAR( eigen.getElement( m1, 0, 0 ), 1, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 1 ), 1, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 2 ), 0, TOLERANCE )

	eigen.rotate( m1, 90, eigen.Vec3( 0, 1, 0 ),  m1, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 0 ), 0, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 0 ), -1, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 2 ), 1, TOLERANCE )

	eigen.scale( m2, eigen.Vec3( 2, 2, 2 ), m2 )
	ASSERT_NEAR( eigen.getElement( m2, 0, 0 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 1, 1 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 2 ), 2, TOLERANCE )

	eigen.identity( m1 )
	eigen.translate( m1, eigen.Vec3( 2, 2, 2 ), m1, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), 2, TOLERANCE	)
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), 2, TOLERANCE )
end

function invTranpTest()
	m1 = eigen.Mat4()

	eigen.translate( m1, eigen.Vec3( 2, 2, 2 ), m1 )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), 2, TOLERANCE )

	eigen.inverseMat( m1, m1 )
	ASSERT_NEAR( eigen.getElement( m1, 0, 3 ), -2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 1, 3 ), -2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 2, 3 ), -2, TOLERANCE )

	eigen.transpose( m1, m1 )
	ASSERT_NEAR( eigen.getElement( m1, 3, 0 ), -2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 3, 1 ), -2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m1, 3, 2 ), -2, TOLERANCE )
	
	elements = m1.elements
	
	EXPECT_NEAR( eigen.getElement( m1, 0, 0 ), elements[1], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 1, 0 ), elements[2], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 2, 0 ), elements[3], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 3, 0 ), elements[4], TOLERANCE )
	
	EXPECT_NEAR( eigen.getElement( m1, 0, 1 ), elements[5], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 1, 1 ), elements[6], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 2, 1 ), elements[7], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 3, 1 ), elements[8], TOLERANCE )	
	
	EXPECT_NEAR( eigen.getElement( m1, 0, 2 ), elements[9], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 1, 2 ), elements[10], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 2, 2 ), elements[11], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 3, 2 ), elements[12], TOLERANCE )
	
	EXPECT_NEAR( eigen.getElement( m1, 0, 3 ), elements[13], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 1, 3 ), elements[14], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 2, 3 ), elements[15], TOLERANCE )
	EXPECT_NEAR( eigen.getElement( m1, 3, 3 ), elements[16], TOLERANCE )
	
end

function elementsAssignTest()
	
	m1 = eigen.Mat4()
	
	local matrix = {}
	
	for i = 1, 16 do
		matrix[i] = 2*i
	end
	
	m1.elements = matrix
	
	EXPECT_NEAR( m1:getElement( 0, 0 ), matrix[1], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 0, 1 ), matrix[5], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 0, 2 ), matrix[9], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 0, 3 ), matrix[13], TOLERANCE )
	
	EXPECT_NEAR( m1:getElement( 1, 0 ), matrix[2], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 1, 1 ), matrix[6], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 1, 2 ), matrix[10], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 1, 3 ), matrix[14], TOLERANCE )	
	
	EXPECT_NEAR( m1:getElement( 2, 0 ), matrix[3], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 2, 1 ), matrix[7], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 2, 2 ), matrix[11], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 2, 3 ), matrix[15], TOLERANCE )
	
	EXPECT_NEAR( m1:getElement( 3, 0 ), matrix[4], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 3, 1 ), matrix[8], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 3, 2 ), matrix[12], TOLERANCE )
	EXPECT_NEAR( m1:getElement( 3, 3 ), matrix[16], TOLERANCE )
	
	local row = ""
	
	elements = m1.elements
	
	for i = 1, 16 do
		EXPECT_NEAR( elements[i], matrix[i], TOLERANCE )
	end

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
	ASSERT_NEAR( x1, x2, TOLERANCE )
	ASSERT_NEAR( y1, y2, TOLERANCE )
	ASSERT_NEAR( z1, z2, TOLERANCE )
end

function fromQuatTest()
	local q = eigen.Quat()
	eigen.setWXYZ( q, 0, 0, 1, 0 )
	m1 = eigen.Mat4()
	eigen.rotate( m1, 180, eigen.Vec3( 0, 1, 0 ), m1 )
	m2 = eigen.fromQuat( q )
	for i = 0,3 do
		for j = 0,3 do
			ASSERT_NEAR( eigen.getElement( m1, i, j ), eigen.getElement( m2, i, j ), TOLERANCE )
		end
	end
end

local lookAtDataZUp = 
{
{xp=0,yp=0,zp=0,xc=0,yc=1,zc=0},
{xp=0,yp=0,zp=0,xc=1,yc=0,zc=0},
{xp=0,yp=0,zp=0,xc=-1,yc=0,zc=0},
{xp=0,yp=0,zp=0,xc=0,yc=-1,zc=0},

{xp=15684,yp=87495,zp=964152,xc=1,yc=1,zc=0},

{xp=-15684,yp=-87495,zp=-964152,xc=1,yc=1,zc=0.8},
{xp=15684,yp=-87495,zp=-964152,xc=1,yc=1,zc=0.1},
{xp=-15684,yp=87495,zp=-964152,xc=1,yc=1,zc=0.3},
{xp=-14584,yp=-87462595,zp=333152,xc=0.00001,yc=1,zc=0.5}



}



function lookatTest()

	for index,value in ipairs(lookAtDataZUp) do

		local pos1 = eigen.Vec3( value.xp, value.yp, value.zp )
		local at1 = eigen.Vec3( value.xc, value.yc, value.zc )
		local up1 = eigen.Vec3( 0, 0, 1 )
		local dirFront1 = eigen.normalize( eigen.subVec( at1, pos1 ) )
		local m1 = eigen.lookAt( pos1, at1, up1 )
		local dirRight1 = eigen.normalize( eigen.crossVec( dirFront1 , eigen.normalize(up1)))
		
		local m2 = eigen.inverseMat( m1 )
		local pos2 = eigen.transform( m2, eigen.Vec3( 0, 0, 0 ) ) --camera position
		local at2 = eigen.transform( m2, eigen.Vec3( 0, 0, -1 ) ) --camera front
		local right2 = eigen.transform( m2, eigen.Vec3( 1, 0, 0 ) ) --camera right
		local dirFront2 = eigen.normalize( eigen.subVec( at2, pos2 ) )
		local dirRight2 = eigen.normalize( eigen.subVec( right2, pos2 ) )
		
		local x1,y1,z1 = eigen.getXYZ( pos1 )
		local x2,y2,z2 = eigen.getXYZ( pos2 )
		ASSERT_NEAR( x1, x2, TOLERANCE )
		ASSERT_NEAR( y1, y2, TOLERANCE )
		ASSERT_NEAR( z1, z2, TOLERANCE )
		
		x1,y1,z1 = eigen.getXYZ( dirFront1 )
		x2,y2,z2 = eigen.getXYZ( dirFront2 )
		ASSERT_NEAR( x1, x2, TOLERANCE )
		ASSERT_NEAR( y1, y2, TOLERANCE )
		ASSERT_NEAR( z1, z2, TOLERANCE )
		
		x1,y1,z1 = eigen.getXYZ( dirRight1 )
		x2,y2,z2 = eigen.getXYZ( dirRight2 )
		ASSERT_NEAR( x1, x2, TOLERANCE )
		ASSERT_NEAR( y1, y2, TOLERANCE )
		ASSERT_NEAR( z1, z2, TOLERANCE )


	end

end

function cameraTest()

	m2 = eigen.frustum( -1, 1, -1, 1, 2, 4 )
	ASSERT_NEAR( eigen.getElement( m2, 0, 0 ), 2, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 2 ), -3, TOLERANCE )
	ASSERT_NEAR( eigen.getElement( m2, 2, 3 ), -8, TOLERANCE )

	m3 = eigen.ortho( -1, 1, -1, 1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 0, 0 ), 1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 2, 2 ), -1 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m3, 2, 3 ), 0 )

	m4 = eigen.perspective( 3.1415 / 2, 1.0, 1.0, 10.0 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 0, 0 ), 72.948834620885 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 2, 2 ), -11/9 )
	ASSERT_DOUBLE_EQ( eigen.getElement( m4, 3, 2 ), -1 )


end

