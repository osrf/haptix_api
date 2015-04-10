//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright 2009-2015 Roboti LLC.  //
//-----------------------------------//

#pragma once

//---------------------------- floating-point definitions -------------------------------

// COMMENT THIS OUT TO SWITCH TO SINGLE PRECISION
#define mjUSEDOUBLE


// floating point data type and minval
#ifdef mjUSEDOUBLE
	typedef double mjtNum;
	#define mjMINVAL	1E-14		// minimum value in any denominator
#else
	typedef float mjtNum;
	#define mjMINVAL	1E-10f
#endif


// global constants
#define mjPI			3.14159265358979323846
#define mjMAXVAL		1E+10		// maximum value in qpos, qvel, qacc
#define mjMINMU			1E-5		// minimum friction coefficient
#define mjMIND			1E-10		// minimum constraint mass
#define mjMAXD			1E+10		// maximum constraint mass 


//---------------------------- sizes ----------------------------------------------------

#define mjNDISABLE		15			// number of disable flags, see mjtDisableBit
#define mjNGEOMTYPES	8			// number of geom types
#define mjNEQDATA		7			// number of eq_data fields
#define mjNDYN			10			// number of actuator dynamics parameters
#define mjNTRN			1			// number of actuator transmission parameters
#define mjNGAIN			5			// number of actuator gain parameters
#define mjNBIAS			3			// number of actuator bias parameters


//---------------------------- primitive types (mjt) ------------------------------------
	
typedef unsigned char mjtByte;		// used for true/false


typedef enum _mjtDisableBit			// disable bitflags
{
	mjDSBL_CONSTRAINT	= 1<<0,		// entire constraint solver
	mjDSBL_EQUALITY		= 1<<1,		// equality constraints
	mjDSBL_FRICTION		= 1<<2,		// joint and tendon friction constraints
	mjDSBL_LIMIT		= 1<<3,		// joint and tendon limit constraints
	mjDSBL_CONTACT		= 1<<4,		// contact constraints
	mjDSBL_MOCAP		= 1<<5,		// copy mocap data into fixed body xpos/quat
	mjDSBL_CIO			= 1<<6,		// modulation of linear cost in solver
	mjDSBL_PASSIVE		= 1<<7,		// passive forces
	mjDSBL_GRAVITY		= 1<<8,		// gravitational forces
	mjDSBL_CLAMPCTRL	= 1<<9,		// clamp control to specified range
	mjDSBL_WARMSTART	= 1<<10,	// warmstart constraint solver
	mjDSBL_ENERGY		= 1<<11,	// compute kinetic and potential energy
	mjDSBL_FILTERPARENT	= 1<<12,	// remove collisions with parent body
	mjDSBL_ACTUATION	= 1<<13,	// apply actuation forces
	mjDSBL_BROADPHASE	= 1<<14		// broad phase collision detection
} mjtDisableBit;


typedef enum _mjtJoint				// type of degree of freedom
{
	mjJNT_FREE			= 0,		// global position and orientation (quat)		(7)
	mjJNT_BALL			= 1,		// orientation (quat) relative to parent		(4)
	mjJNT_SLIDE			= 2,		// sliding distance along body-fixed axis		(1)
	mjJNT_HINGE			= 3			// rotation angle (rad) around body-fixed axis	(1)
} mjtJoint;


typedef enum _mjtGeom				// type of geometric shape
{
	// standard geom types
	mjGEOM_PLANE		= 0,		// plane
	mjGEOM_HFIELD		= 1,		// height field
	mjGEOM_SPHERE		= 2,		// sphere
	mjGEOM_CAPSULE		= 3,		// capsule
	mjGEOM_ELLIPSOID	= 4,		// ellipsoid
	mjGEOM_CYLINDER		= 5,		// cylinder
	mjGEOM_BOX			= 6,		// box
	mjGEOM_MESH			= 7,		// mesh

	// rendering-only geom types: not used in mjModel, not counted in mjNGEOMTYPES
	mjGEOM_ARROW		= 100,		// arrow
	mjGEOM_ARROW1		= 101,		// arrow without wedges
	mjGEOM_ARROW2		= 102,		// arrow in both directions
	mjGEOM_TORQUE		= 103,		// torque rendering
	mjGEOM_LABEL		= 104,		// text label
	mjGEOM_TUBE			= 105,		// open-face cylinder
	mjGEOM_NONE			= 1001		// missing geom type
} mjtGeom;


typedef enum _mjtIntegrator			// integrator mode
{
	mjINT_EULER = 0,				// semi-implicit Euler
	mjINT_RK4						// 4th-order Runge Kutta
} mjtIntegrator;


typedef enum _mjtCollision			// collision mode for selecting geom pairs
{
	mjCOL_ALL = 0,					// test precomputed and dynamic pairs
	mjCOL_PAIR,				        // test predefined pairs only
	mjCOL_DYNAMIC					// test dynamic pairs only
} mjtCollision;


typedef enum _mjtSolver				// constraint solver algorithm
{
	mjSOL_NONE = 0,					// do not run solver
	mjSOL_SPARSE,					// sparse pyramidal cone, PCG solver
	mjSOL_DENSE_PCG,				// dense pyramidal cone, PCG solver
	mjSOL_DENSE_PGS,				// dense pyramidal cone, PGS solver
	mjSOL_ELLIPTIC					// dense elliptic cone, generalized PGS solver
} mjtSolver;


typedef enum _mjtEq					// type of equality constraint
{
	mjEQ_CONNECT = 0,				// connect two bodies at a point (ball joint)
	mjEQ_WELD,						// fix relative position and orientation of two bodies
	mjEQ_JOINT,						// couple the values of two scalar joints with cubic
	mjEQ_TENDON,					// couple the lengths of two tendons with cubic
	mjEQ_DISTANCE					// fix the contact distance betweent two geoms
} mjtEq;


typedef enum _mjtWrap				// type of tendon wrap object
{
	mjWRAP_NONE = 0,				// null object
	mjWRAP_JOINT,					// constant moment arm
	mjWRAP_PULLEY,					// pulley used to split tendon
	mjWRAP_SITE,					// pass through site
	mjWRAP_SPHERE,					// wrap around sphere
	mjWRAP_CYLINDER					// wrap around (infinite) cylinder
} mjtWrap;


typedef enum _mjtTrn				// type of actuator transmission
{
	mjTRN_JOINT = 0,				// apply force on joint
	mjTRN_SLIDERCRANK,				// apply force via slider-crank linkage
	mjTRN_TENDON,					// apply force on tendon
	mjTRN_UNDEFINED = 1000			// undefined transmission type
} mjtTrn;


typedef enum _mjtDyn				// type of actuator dynamics
{
	mjDYN_NONE = 0,					// no internal dynamics; ctrl specifies force
	mjDYN_INTEGRATOR,				// integrator: da/dt = u
	mjDYN_FILTER,					// linear filter: da/dt = (u-a) / tau
	mjDYN_USER						// user-defined dynamics type
} mjtDyn;


typedef enum _mjtGain				// type of actuator gain
{
	mjGAIN_FIXED = 0,				// fixed gain
	mjGAIN_USER						// user-defined gain type
} mjtGain;


typedef enum _mjtBias				// type of actuator bias
{
	mjBIAS_NONE = 0,				// no bias
	mjBIAS_AFFINE,					// const + kp*length + kv*velocity
	mjBIAS_USER						// user-defined bias type
} mjtBias;


typedef enum _mjtObj				// type of MujoCo object
{
	mjOBJ_UNKNOWN = 0,				// unknown object type
	mjOBJ_BODY,						// body
	mjOBJ_JOINT,					// joint
	mjOBJ_DOF,						// dof
	mjOBJ_GEOM,						// geom
	mjOBJ_SITE,						// site
	mjOBJ_CAMERA,					// camera
	mjOBJ_LIGHT,					// light
	mjOBJ_MESH,						// mesh
	mjOBJ_HFIELD,					// heightfield
	mjOBJ_TEXTURE,					// texture
	mjOBJ_MATERIAL,					// material for rendering
	mjOBJ_PAIR,						// geom pair to include
	mjOBJ_EXCLUDE,					// body pair to exclude
	mjOBJ_CIO,						// cio body pair
	mjOBJ_EQUALITY,					// equality constraint
	mjOBJ_TENDON,					// tendon
	mjOBJ_ACTUATOR,					// actuator
	mjOBJ_SENSOR,					// sensor
	mjOBJ_NUMERIC,					// numeric
	mjOBJ_TEXT						// text
} mjtObj;


typedef enum _mjtConstraint			// type of constraint
{
	mjCNSTR_EQUALITY = 0,			// equality constraint
	mjCNSTR_FRICTION_DOF,			// dof friction
	mjCNSTR_FRICTION_TENDON,		// tendon friction
	mjCNSTR_LIMIT_JOINT,			// joint limit
	mjCNSTR_LIMIT_TENDON,			// tendon limit
	mjCNSTR_CONTACT_FRICTIONLESS,	// frictionless contact
	mjCNSTR_CONTACT_PYRAMIDAL,		// frictional contact, pyramidal friction cone
	mjCNSTR_CONTACT_ELLIPTIC		// frictional contact, elliptic friction cone
} mjtConstraint;


typedef enum _mjtSensor				// type of sensor
{
	mjSENS_TOUCH = 0,				// touch sensor (defined by site)
	mjSENS_IMU,						// IMU sensor (defined by site)
	mjSENS_FT,						// force-torque sensor (defined by child body)
	mjSENS_JOINTPOS,				// joint position
	mjSENS_JOINTVEL,				// joint velocity
	mjSENS_TENDONPOS,				// tendon position
	mjSENS_TENDONVEL,				// tendon velocity
	mjSENS_ACTUATORPOS,				// actuator position
	mjSENS_ACTUATORVEL,				// actuator velocity
	mjSENS_ACTUATORFRC				// actuator force
} mjtSensor;



//------------------------------ mjOption -----------------------------------------------

struct _mjOption					// physics options
{
	// integrator parameters
	mjtNum timestep;				// timestep
	mjtNum tolerance;				// error tolerance for adaptive integrator

	// physical constants
	mjtNum gravity[3];				// gravitational acceleration
	mjtNum wind[3];					// wind (for lift, drag and viscosity)
	mjtNum density;					// density of medium
	mjtNum viscosity;				// viscosity of medium

	// rescale parameters:  modified = s0 + s1*original
	mjtNum s_margin[2];				// rescale margin
	mjtNum s_solgap[2];				// rescale solvergap
	mjtNum s_solprm0[2];			// rescale solver parameter 0
	mjtNum s_solprm1[2];			// rescale solver parameter 1
	mjtNum s_solprm2[2];			// rescale solver parameter 2
	mjtNum s_solprm3[2];			// rescale solver parameter 3

	// discrete settings
	int integrator;					// integration mode (mjtIntegrator)
	int collision;					// collision mode (mjtCollision)
	int solver;						// solver mode (mjtSolver)
	int iterations;					// number of solver iterations
	int disableflags;				// bit flags for disabling engine features
};
typedef struct _mjOption mjOption;


//------------------------------ mjVisual -----------------------------------------------

struct _mjVisual					// visualization options
{
	struct							// global parameters
	{
		float fovy;					// y-field of view (deg) for free camera
		float ipd;					// inter-pupilary distance for free camera
		float linewidth;			// line width for wireframe rendering
		float glow;					// glow coefficient for selected body
	} global;

	struct							// rendering quality
	{
		int   shadowsize;			// size of shadowmap texture
		int   numSlices;			// number of slices for Glu drawing
		int   numStacks;			// number of stacks for Glu drawing
		int   numArrows;			// number of arrows for torque rendering
									// box and plane division ???
	} quality;

	struct							// head light
	{
		float ambient[3];			// ambient rgb (alpha=1)
		float diffuse[3];			// diffuse rgb (alpha=1)
		float specular[3];			// specular rgb (alpha=1)
		int   active;				// is headlight active
	} headlight;

	struct							// mapping
	{
		float stiffness;			// mouse perturbation stiffness (space->force)
		float force;				// from force units to space units
		float torque;				// from torque units to space units
		float alpha;				// scale geom alphas when transparency is enabled
	} map;

	struct							// scale of decor elements relative to mean body size
	{
		float forcewidth;			// width of force arrow
		float contactwidth;			// contact width
		float contactheight;		// contact height
		float connect;				// autoconnect capsule width
		float com;					// com radius
		float selectpoint;			// selection point
		float jointlength;			// joint length
		float jointwidth;			// joint width
		float actuatorlength;		// actuator length
		float actuatorwidth;		// actuator width
		float framelength;			// bodyframe axis length
		float framewidth;			// bodyframe axis width
		float constraint;			// constraint width
		float slidercrank;			// slidercrank width
	} scale;

	struct							// color of decor elements
	{
		float force[4];				// external force
		float inertia[4];			// inertia box
		float joint[4];				// joint
		float actuator[4];			// actuator
		float com[4];				// center of mass
		float selectpoint[4];		// selection point
		float connect[4];			// auto connect
		float contactpoint[4];		// contact point
		float contactforce[4];		// contact force
		float contactfriction[4];	// contact friction force
		float contacttorque[4];		// contact torque
		float constraint[4];		// constraint
		float slidercrank[4];		// slidercrank
		float crankbroken[4];		// used when crank must be stretched/broken
	} rgba;
};
typedef struct _mjVisual mjVisual;


//------------------------------ mjStatistic --------------------------------------------

struct _mjStatistic					// model statistics (in qpos0)
{
	mjtNum extent;					// spatial extent
	mjtNum center[3];				// center of boundng box
	mjtNum meanmass;				// mean body mass
	mjtNum meansize;				// mean body size
};
typedef struct _mjStatistic mjStatistic;


//---------------------------------- mjModel --------------------------------------------

struct _mjModel
{
	// ------------------------------- sizes

	// sizes needed at mjModel construction
	int nq;							// number of generalized coordinates = dim(qpos)
	int nv;							// number of degrees of freedom = dim(qvel)
	int nu;							// number of actuators/controls = dim(ctrl)
	int na;							// number of activation states = dim(act)
	int nbody;						// number of bodies
	int njnt;						// number of joints
	int ngeom;						// number of geoms
	int nsite;						// number of sites
	int ncam;						// number of cameras
	int nlight;						// number of lights
	int nmesh;						// number of meshes
	int nmeshvert;					// number of vertices in all meshes
	int nmeshface;					// number of triangular faces in all meshes
	int nmeshgraph;					// number of ints in mesh auxiliary data
	int nhfield;					// number of heightfields
	int nhfielddata;				// number of data points in all heightfields
	int ntex;						// number of textures
	int ntexdata;					// number of rgb triplets in all textures
	int nmat;						// number of materials
	int	npair;						// number of predefined geom pairs
	int	nexclude;					// number of excluded geom pairs
	int ncio;						// number of cio body pairs
	int neq;						// number of equality constraints
	int ntendon;					// number of tendons
	int nwrap;						// number of wrap objects in all tendon paths
	int nsensor;					// number of sensors
	int nnumeric;					// number of numeric user fields
	int nnumericdata;				// number of mjtNums in all numeric fields
	int ntext;						// number of text user fields
	int ntextdata;					// number of mjtBytes in all text fields
	int nkey;						// number of keyframes
	int nuser_body;					// number of mjtNums in body_user
	int nuser_jnt;					// number of mjtNums in jnt_user
	int nuser_geom;					// number of mjtNums in geom_user
	int nuser_site;					// number of mjtNums in site_user
	int nuser_tendon;				// number of mjtNums in tendon_user
	int nuser_actuator;				// number of mjtNums in actuator_user
	int nuser_sensor;				// number of mjtNums in sensor_user
	int nnames;						// number of chars in all names

	// sizes set after mjModel construction (only affect mjData)
	int nM;							// number of non-zeros in sparse inertia matrix
	int nemax;						// number of potential equality-constraint rows
	int njmax;						// number of available rows in constraint Jacobian
	int	nconmax;					// number of potential contacts in contact list
	int nstack;						// number of fields in mjData stack
	int nuserdata;					// number of extra fields in mjData
	int nciodim;					// number of cio contact force dimensions ???
	int nmocap;						// number of mocap bodies
	int nsensordata;				// number of fields in sensor data vector

	int nbuffer;					// number of bytes in buffer

	// ------------------------------- options and statistics

	mjOption opt;					// physics options
	mjVisual vis;					// visualization options
	mjStatistic stat;				// model statistics

	// ------------------------------- buffers

	// main buffer
	void*	  buffer;				// main buffer; all pointers point in it	(nbuffer)

	// default generalized coordinates
	mjtNum*	  qpos0;				// qpos values at default pose				(nq x 1)
	mjtNum*	  qpos_spring;			// reference pose for springs				(nq x 1)

	// bodies
	int*	  body_parentid;		// id of body's parent						(nbody x 1)
	int*  	  body_rootid;			// id of root above body					(nbody x 1)
	int*  	  body_weldid;			// id of body that this body is welded to	(nbody x 1)
	int*  	  body_mocapid;			// id of mocap data; -1: none				(nbody x 1)
	int*	  body_jntnum;			// number of joints for this body			(nbody x 1)
	int*	  body_jntadr;			// start addr of joints; -1: no joints		(nbody x 1)
	int*	  body_dofnum;			// number of motion degrees of freedom		(nbody x 1)
	int*	  body_dofadr;			// start addr of dofs; -1: no dofs			(nbody x 1)
	int*	  body_geomnum;			// number of geoms							(nbody x 1)
	int*	  body_geomadr;			// start addr of geoms; -1: no geoms		(nbody x 1)
	mjtNum*   body_pos;				// position offset rel. to parent body		(nbody x 3)
	mjtNum*   body_quat;			// orientation offset rel. to parent body	(nbody x 4)
	mjtNum*   body_ipos;			// local position of center of mass			(nbody x 3)
	mjtNum*   body_iquat;			// local orientation of inertia ellipsoid	(nbody x 4)
	mjtNum*   body_mass;			// mass										(nbody x 1)
	mjtNum*   body_inertia;			// diagonal inertia in ipos/iquat frame		(nbody x 3)
	mjtNum*   body_invweight0;		// mean inv inert in qpos0 (trn, rot)		(nbody x 2)
	mjtNum*   body_user;			// user data								(nbody x nuser_body)

	// joints
	int*	  jnt_type;				// type of joint (mjtJoint)					(njnt x 1)
	int*	  jnt_qposadr;			// start addr in 'qpos' for joint's data	(njnt x 1)
	int*	  jnt_dofadr;			// start addr in 'qvel' for joint's data	(njnt x 1)
	int*	  jnt_bodyid;			// id of joint's body						(njnt x 1)
	mjtByte*  jnt_limited;			// does joint have limits					(njnt x 1)
	mjtNum*	  jnt_solprm;			// constraint solver parameters: limit		(njnt x 4)
	mjtNum*	  jnt_pos;				// local anchor position					(njnt x 3)
	mjtNum*	  jnt_axis;				// local joint axis							(njnt x 3)
	mjtNum*	  jnt_stiffness;		// stiffness coefficient					(njnt x 1)
	mjtNum*	  jnt_range;			// joint limits								(njnt x 2)
	mjtNum*	  jnt_margin;			// min distance for limit detection			(njnt x 1)
	mjtNum*	  jnt_user;				// user data								(njnt x nuser_jnt)

	// dofs
	int*	  dof_bodyid;			// id of dof's body							(nv x 1)
	int*	  dof_jntid;			// id of dof's joint						(nv x 1)
	int*	  dof_parentid;			// id of dof's parent; -1: none				(nv x 1)
	int*	  dof_Madr;				// dof address in M-diagonal				(nv x 1)
	mjtByte*  dof_frictional;		// does dof have friction					(nv x 1)
	mjtNum*	  dof_solprm;			// constraint solver parameters: friction	(nv x 4)
	mjtNum*   dof_frictionloss;		// dof friction loss						(nv x 1)
	mjtNum*   dof_armature;			// dof armature inertia/mass				(nv x 1)
	mjtNum*	  dof_damping;			// damping coefficient						(nv x 1)
	mjtNum*   dof_invweight0;		// inv. diag. inertia in qpos0				(nv x 1)

	// geoms
	int*	  geom_type;			// geometric type (mjtGeom)					(ngeom x 1)
	int*	  geom_contype;			// geom contact type						(ngeom x 1)
	int*	  geom_conaffinity;		// geom contact affinity					(ngeom x 1)
	int*	  geom_condim;			// contact dimensionality (1, 3, 4, 6)		(ngeom x 1)
	int*	  geom_bodyid;			// id of geom's body						(ngeom x 1)
	int*	  geom_dataid;			// id of geom's mesh or hfield (-1: none)	(ngeom x 1)
	int*	  geom_matid;			// material id for rendering				(ngeom x 1)
	int*	  geom_group;			// group for show/hide						(ngeom x 1)
	mjtNum*	  geom_solprm;			// constraint solver parameters: contact	(ngeom x 4)
	mjtNum*   geom_size;			// geom-specific size parameters			(ngeom x 3)
	mjtNum*   geom_rbound;			// radius of bounding sphere				(ngeom x 1)
	mjtNum*   geom_pos;				// local position offset rel. to body		(ngeom x 3)
	mjtNum*   geom_quat;			// local orientation offset rel. to body	(ngeom x 4)
	mjtNum*   geom_friction;		// friction for (slide, spin, roll)			(ngeom x 3)
	mjtNum*   geom_margin;			// detect contact if dist<margin			(ngeom x 1)
	mjtNum*   geom_solgap;			// include in solver if dist<margin-gap 	(ngeom x 1)
	mjtNum*   geom_user;			// user data								(ngeom x nuser_geom)
	float*    geom_rgba;			// rgba when material is omitted			(ngeom x 4)

	// sites
	int*	  site_geomtype;		// geom type for rendering (mjtGeom)		(nsite x 1)
	int*	  site_bodyid;			// id of site's body						(nsite x 1)
	int*	  site_matid;			// material id for rendering				(nsite x 1)
	int*	  site_group;			// group for show/hide						(nsite x 1)
	mjtNum*   site_geomsize;		// geom size for rendering					(nsite x 3)
	mjtNum*   site_pos;				// local position offset rel. to body		(nsite x 3)
	mjtNum*   site_quat;			// local orientation offset rel. to body	(nsite x 4)
	mjtNum*   site_user;			// user data								(nsite x nuser_site)
	float*    site_rgba;			// rgba when material is omitted			(nsite x 4)

	// cameras
	int*      cam_bodyid;			// id of camera's body						(ncam x 1)
	mjtNum*	  cam_pos;				// position rel. to body frame				(ncam x 3)
	mjtNum*	  cam_quat;				// orientation rel. to body frame			(ncam x 4)
	mjtNum*   cam_fovy;				// y-field of view (deg)					(ncam x 1)
	mjtNum*   cam_ipd;				// inter-pupilary distance					(ncam x 1)

	// lights
	int*      light_bodyid;			// id of camera's body						(nlight x 1)
	mjtByte*  light_directional;	// directional light						(nlight x 1)
	mjtByte*  light_castshadow;		// does light cast shadows					(nlight x 1)
	mjtByte*  light_active;			// is light on								(nlight x 1)
	mjtNum*	  light_pos;			// position rel. to body frame				(nlight x 3)
	mjtNum*	  light_dir;			// direction rel. to body frame				(nlight x 3)
	float*    light_attenuation;	// OpenGL attenuation (quadratic model)		(nlight x 3)
	float*    light_cutoff;			// OpenGL cutoff							(nlight x 1)
	float*    light_exponent;		// OpenGL exponent							(nlight x 1)
	float*    light_ambient;		// ambient rgb (alpha=1)					(nlight x 3)
	float*    light_diffuse;		// diffuse rgb (alpha=1)					(nlight x 3)
	float*    light_specular;		// specular rgb (alpha=1)					(nlight x 3)

	// meshes
	int*	  mesh_faceadr;			// first face address						(nmesh x 1)
	int*	  mesh_facenum;			// number of faces							(nmesh x 1)
	int*	  mesh_vertadr;			// first vertex address						(nmesh x 1)
	int*	  mesh_vertnum;			// number of vertices						(nmesh x 1)
	int*	  mesh_graphadr;		// graph data address; -1: no graph			(nmesh x 1)
	float*	  mesh_vert;			// vertex data for all meshes				(nmeshvert x 3)
	float*	  mesh_normal;			// vertex normal data for all meshes		(nmeshvert x 3)
	int*	  mesh_face;			// triangle face data						(nmeshface x 3)
	int*	  mesh_graph;			// convex graph data						(nmeshgraph x 1)

	// height fields
	int*	  hfield_nrow;			// number of rows in grid					(nhfield x 1)
	int*	  hfield_ncol;			// number of columns in grid				(nhfield x 1)
	int*	  hfield_adr;			// address in hfield_data					(nhfield x 1)
	float*	  hfield_data;			// elevation data							(nhfielddata x 1)

	// textures
	int*	  tex_nrow;				// number of rows in texture image			(ntex x 1)
	int*	  tex_ncol;				// number of columns in texture image		(ntex x 1)
	int*	  tex_adr;				// address in rgb							(ntex x 1)
	mjtByte*  tex_rgb;				// rgb (alpha = 1)							(ntexdata x 1)

	// materials
	int*	  mat_texid;			// texture id; -1: none						(nmat x 1)
	float*	  mat_emission;			// emission (x rgb)							(nmat x 1)
	float*	  mat_specular;			// specular (x white)						(nmat x 1)
	float*	  mat_shininess;		// shininess coef							(nmat x 1)
	float*	  mat_reflectance;		// reflectance (0: disable)					(nmat x 1)
	float*    mat_rgba;				// rgba										(nmat x 4)

	// predefined geom pairs for collision detection; has precedence over exclude
	int*	  pair_dim;				// contact dimensionality					(npair x 1)
	int*	  pair_geom1;			// id of geom1								(npair x 1)
	int*	  pair_geom2;			// id of geom2								(npair x 1)
	int*	  pair_signature;		// (body1+1)<<16 + body2+1					(npair x 1)
	mjtNum*	  pair_solprm;			// constraint solver parameters: contact	(npair x 4)
	mjtNum*   pair_margin;			// detect contact if dist<margin			(npair x 1)
	mjtNum*   pair_solgap;			// include in solver if dist<margin-gap 	(npair x 1)
	mjtNum*	  pair_friction;		// tangent1, 2, spin, roll1, 2				(npair x 5)

	// excluded body pairs for collision detection
	int*	  exclude_signature;	// (body1+1)<<16 + body2+1					(nexclude x 1)

	// cio
	int*	  cio_body1;			// body1 id									(ncio x 1)
	int*	  cio_body2;			// body2 id									(ncio x 1)
	mjtNum*	  cio_softmin;			// parameter for cio force distribution		(ncio x 1)

	// equality constraints
	int*	  eq_type;				// constraint type (mjtEq)					(neq x 1)
	int*	  eq_obj1id;			// id of object 1							(neq x 1)
	int*	  eq_obj2id;			// id of object 2							(neq x 1)
	mjtByte*  eq_active;			// enable/disable constraint				(neq x 1)
	mjtNum*	  eq_solprm;			// constraint solver parameters: equality	(neq x 4)
	mjtNum*	  eq_data;				// numeric data for constraint				(neq x mjNEQDATA)

	// tendons
	int*      tendon_adr;			// address of first object in tendon's path (ntendon x 1)
	int*	  tendon_num;			// number of objects in tendon's path		(ntendon x 1)
	int*	  tendon_matid;			// material id for rendering				(ntendon x 1)
	mjtByte*  tendon_limited;		// does tendon have length limits			(ntendon x 1)
	mjtByte*  tendon_frictional;	// does tendon have friction				(ntendon x 1)
	mjtNum*	  tendon_width;			// width for rendering						(ntendon x 1)
	mjtNum*	  tendon_solprm_lim;	// constraint solver parameters: limit		(ntendon x 4)
	mjtNum*	  tendon_solprm_fri;	// constraint solver parameters: friction	(ntendon x 4)
	mjtNum*	  tendon_range;			// tendon length limits						(ntendon x 2)
	mjtNum*	  tendon_margin;		// min distance for limit detection			(ntendon x 1)
	mjtNum*	  tendon_stiffness;		// stiffness coefficient					(ntendon x 1)
	mjtNum*	  tendon_damping;		// damping coefficient						(ntendon x 1)
	mjtNum*	  tendon_frictionloss;	// loss due to friction						(ntendon x 1)
	mjtNum*	  tendon_lengthspring;	// tendon length in qpos_spring				(ntendon x 1)
	mjtNum*	  tendon_length0;		// tendon length in qpos0					(ntendon x 1)
	mjtNum*	  tendon_invweight0;	// inv. weight in qpos0						(ntendon x 1)
	mjtNum*	  tendon_user;			// user data								(ntendon x nuser_tendon)
	float*    tendon_rgba;			// rgba when material is omitted			(ntendon x 4)

	// list of all wrap objects in tendon paths
	int*	  wrap_type;			// wrap object type (mjtWrap)				(nwrap x 1)
	int*	  wrap_objid;			// object id: geom, site, joint				(nwrap x 1)
	mjtNum*	  wrap_prm;				// divisor, joint coef, or site id			(nwrap x 1)

	// actuators
	int*	  actuator_dyntype;		// dynamics type (mjtDyn)					(nu x 1)
	int*	  actuator_trntype;		// transmission type (mjtTrn)				(nu x 1)
	int*	  actuator_gaintype;	// gain type (mjtGain)						(nu x 1)
	int*	  actuator_biastype;	// bias type (mjtBias)						(nu x 1)
	int*	  actuator_trnid;		// transmission id: joint, tendon, site		(nu x 2)
	mjtByte*  actuator_ctrllimited; // is control limited						(nu x 1)
	mjtByte*  actuator_forcelimited;// is force limited							(nu x 1)
	mjtNum*	  actuator_dynprm;		// dynamics parameters						(nu x mjNDYN)
	mjtNum*	  actuator_trnprm;		// transmission parameters					(nu x mjNTRN)
	mjtNum*	  actuator_gainprm;		// gain parameters							(nu x mjNGAIN)
	mjtNum*	  actuator_biasprm;		// bias parameters							(nu x mjNBIAS)
	mjtNum*	  actuator_ctrlrange;	// range of controls						(nu x 2)
	mjtNum*	  actuator_forcerange;	// range of forces							(nu x 2)
	mjtNum*	  actuator_invweight0;	// inv. weight in qpos0						(nu x 1)
	mjtNum*	  actuator_length0;		// actuator length in qpos0					(nu x 1)
	mjtNum*	  actuator_lengthrange;	// ... not yet implemented ???				(nu x 2)
	mjtNum*	  actuator_user;		// user data								(nu x nuser_actuator)

	// sensors
	int*	  sensor_type;			// sensor type (mjtSensor)					(nsensor x 1)
	int*	  sensor_objid;			// id of sensorized object					(nsensor x 1)
	int*	  sensor_dim;			// number of scalar outputs					(nsensor x 1)
	int*	  sensor_adr;			// address in sensor array					(nsensor x 1)
	mjtNum*   sensor_user;			// user data								(nsensor x nuser_sensor)

	// custom numeric fields
	int*	  numeric_adr;			// address of field in numeric_data			(nnumeric x 1)
	int*	  numeric_size;			// size of numeric field					(nnumeric x 1)
	mjtNum*   numeric_data;			// array of all numeric fields				(nnumericdata x 1)

	// custom text fields
	int*	  text_adr;				// address of text in text_data				(ntext x 1)
	char*	  text_data;			// array of all custom texts				(ntextdata x 1)

	// keyframes
	mjtNum*	  key_time;				// key time									(nkey x 1)
	mjtNum*	  key_qpos;				// key position								(nkey x nq)
	mjtNum*	  key_qvel;				// key velocity								(nkey x nv)
	mjtNum*	  key_act;				// key activation							(nkey x na)

	// names
	int*	  name_bodyadr;			// body name pointers					    (nbody x 1)
	int*	  name_jntadr;			// joint name pointers					    (njnt x 1)
	int*	  name_geomadr;			// geom name pointers						(ngeom x 1)
	int*	  name_siteadr;			// site name pointers						(nsite x 1)
	int*	  name_camadr;			// camera name pointers						(ncam x 1)
	int*	  name_lightadr;		// light name pointers						(nlight x 1)
	int*	  name_meshadr;			// mesh name pointers						(nmesh x 1)
	int*	  name_hfieldadr;		// hfield name pointers						(nhfield x 1)
	int*	  name_texadr;			// texture name pointers					(ntex x 1)
	int*	  name_matadr;			// material name pointers					(nmat x 1)
	int*	  name_eqadr;			// equality constraint name pointers		(neq x 1)
	int*	  name_tendonadr;		// tendon name pointers					    (ntendon x 1)
	int*	  name_actuatoradr;		// actuator name pointers				    (nu x 1)
	int*	  name_sensoradr;		// sensor name pointers						(nsensor x 1)
	int*	  name_numericadr;		// numeric name pointers				    (nnumeric x 1)
	int*	  name_textadr;			// text name pointers					    (ntext x 1)
	char*	  names;				// names of all objects, 0-terminated		(nnames x 1)
};
typedef struct _mjModel mjModel;
