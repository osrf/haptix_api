C API for Haptix project

Some thoughts from Emo
===========

The same C struct should be able to support different models. 
And ideally we will allocate it statically. This means we have to
define maximum sizes for the different arrays, and have (read-only)
fields specifying the actual size of the arrays for a given model.
The latter should perhaps be stored in a separate Info struct.
Something like:

struct hxInfo
{
	int njnt;
	int nobj;
	char jnt_name[JNT_MAX][NAME_MAX];
	char obj_name[OBJ_MAX][NAME_MAX];
	// other constant model properties
};

struct hxState
{
	double jnt_pos[JNT_MAX];
	double jnt_vel[JNT_MAX];
	double obj_pos[OBJ_MAX][3];
	double obj_quat[OBJ_MAX][4];
	double obj_linvel[OBJ_MAX][3];
	double obj_angvel[OBJ_MAX][3];
	// other state variables
};

By "object" (obj) I mean a root body in the kinematic tree.
There is always one such body corresponding to the base of the
robot. But there could be other objects in the environment
that the robot is interacting with. Since this project
does not have a computer vision component, we should provide
to the user the ground truth for all object states.