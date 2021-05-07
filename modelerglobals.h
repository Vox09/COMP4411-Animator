#ifndef _MODELER_GLOBALS_H
#define _MODELER_GLOBALS_H

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif

// This is a list of the controls for the SampleModel
// We'll use these constants to access the values 
// of the controls from the user interface.
#ifdef SAMPLE
enum SampleModelControls
{ 
	XPOS, YPOS, ZPOS, HEIGHT, ROTATE, NUMCONTROLS
};

#else
enum TomModelControls
{ 
	LVL,  
	//light
	LIGHT_X, LIGHT_Y, LIGHT_Z,
	LIGHT_INTENSITY,
	//change component
	BODYCHANGE, HEADCHANGE,
	//
	MOOD,
	//inverse kinematics
	IK_X, IK_Y, IK_Z, 
	IK_CSTR_1L, IK_CSTR_1H, 
	IK_CSTR_2L, IK_CSTR_2H, 
	IK_CSTR_3L, IK_CSTR_3H, 
	// particle count
	PARTICLE_COUNT,
	NUMCONTROLS
};
#endif 

// Colors
#define COLOR_RED		1.0f, 0.0f, 0.0f
#define COLOR_GREEN		0.0f, 1.0f, 0.0f
#define COLOR_BLUE		0.0f, 0.0f, 1.0f
#define COLOR_LIGHT_GREY		0.8f, 0.8f, 0.8f
#define COLOR_DARK_GREY			0.1f, 0.1f, 0.2f

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))

#endif