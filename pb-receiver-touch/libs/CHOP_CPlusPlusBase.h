/* Shared Use License: This file is owned by Derivative Inc. (Derivative) and
 * can only be used, and/or modified for use, in conjunction with 
 * Derivative's TouchDesigner software, and only if you are a licensee who has
 * accepted Derivative's TouchDesigner license or assignment agreement (which
 * also govern the use of this file).  You may share a modified version of this
 * file with another authorized licensee of Derivative's TouchDesigner software.
 * Otherwise, no redistribution or sharing of this file, with or without
 * modification, is permitted.
 */

/*
 * Produced by:
 *
 * 				Derivative Inc
 *				401 Richmond Street West, Unit 386
 *				Toronto, Ontario
 *				Canada   M5V 3A8
 *				416-591-3555
 *
 * NAME:				CHOP_CPlusPlusBase.h 
 *
 *
 *	Do not edit this file directly!
 *	Make a subclass of CHOP_CPlusPlusBase instead, and add your own 
 *	data/functions.

 *	Derivative Developers:: Make sure the virtual function order
 *	stays the same, otherwise changes won't be backwards compatible
 */

#ifndef __CHOP_CPlusPlusBase__
#define __CHOP_CPlusPlusBase__

#include "CPlusPlus_Common.h"


class CHOP_CPlusPlusBase;

// Define for the current API version that this sample code is made for.
// To upgrade to a newer version, replace the files
// CHOP_CPlusPlusBase.h
// CPlusPlus_Common.h
// from the samples folder in a newer TouchDesigner installation.
// You may need to upgrade your plugin code in that case, to match
// the new API requirements
const int CHOPCPlusPlusAPIVersion = 8;

struct CHOP_PluginInfo
{
public:

	// Must be set to CHOPCPlusPlusAPIVersion in FillCHOPPluginInfo
	int32_t			apiVersion = 0;

	int32_t			reserved[100];


	// Information used to describe this plugin as a custom OP.
	OP_CustomOPInfo	customOPInfo;


	int32_t			reserved2[20];

};



// These are the definitions for the C-functions that are used to
// load the library and create instances of the object you define
typedef void (__cdecl *FILLCHOPPLUGININFO)(CHOP_PluginInfo *info);
typedef CHOP_CPlusPlusBase* (__cdecl *CREATECHOPINSTANCE)(const OP_NodeInfo*);
typedef void (__cdecl *DESTROYCHOPINSTANCE)(CHOP_CPlusPlusBase*);


class CHOP_GeneralInfo
{
public:
	// Set this to true if you want the CHOP to cook every frame, even
	// if none of it's inputs/parameters are changing
	// DEFAULT: false

	bool			cookEveryFrame;

	// Set this to true if you want the CHOP to cook every frame, but only
	// if someone asks for it to cook. So if nobody is using the output from
	// the CHOP, it won't cook. This is difereent from 'cookEveryFrame'
	// since that will cause it to cook every frame no matter what.

	bool			cookEveryFrameIfAsked;

	// Set this to true if you will be outputting a timeslice
	// Outputting a timeslice means the number of samples in the CHOP will 
	// be determined by the number of frames that have elapsed since the last 
	// time TouchDesigner cooked (it will be more than one in cases where it's 
	// running slower than the target cook rate), the playbar framerate and 
	// the sample rate of the CHOP.
	// For example if you are outputting the CHOP 120hz sample rate, 
	// TouchDesigner is running at 60 hz cookrate, and you missed a frame last cook
	// then on this cook the number of sampels of the output of this CHOP will
	// be 4 samples. I.e (120 / 60) * number of playbar frames to output.
	// If this isn't set then you specify the number of sample in the CHOP using
	// the getOutputInfo() function
	// DEFAULT: false

	bool			timeslice;

	// If you are returning 'false' from getOutputInfo, this index will 
	// specify the CHOP input whos attribues you will match 
	// (channel names, length, sample rate etc.)
	// DEFAULT : 0

	int32_t			inputMatchIndex;


	int32_t			reserved[20];
};



class CHOP_OutputInfo
{
public:

	// The number of channels you want to output

	int32_t			numChannels;


	// If you arn't outputting a timeslice, specify the number of samples here

	int32_t			numSamples;


	// if you arn't outputting a timeslice, specify the start index
	// of the channels here. This is the 'Start' you see when you
	// middle click on a CHOP

	uint32_t		startIndex;


	// Specify the sample rate of the channel data
	// DEFAULT : whatever the timeline FPS is ($FPS)

	float			sampleRate;


	void*			reserved1;


	int32_t			reserved[20];

};





class CHOP_Output
{
public:
	CHOP_Output(int32_t nc, int32_t l, float s, uint32_t st,
					float **cs, const char** ns):
											numChannels(nc),
											numSamples(l),
											sampleRate(s),
											startIndex(st),
											channels(cs),
											names(ns)
	{
	}

	// Info about what you are expected to output
	const int32_t	numChannels;
	const int32_t	numSamples;
	const float		sampleRate;
	const uint32_t	startIndex;

	// This is an array of const char* that tells you the channel names
	// of the channels you are providing values for. It's 'numChannels' long. 
	// E.g names[3] is the name of the 4th channel
	const char** const 	names;

	// This is an array of float arrays that is already allocated for you.
	// Fill it with the data you want outputted for this CHOP.
	// The length of the array is 'numChannels',
	// While the length of each of the array entries is 'numSamples'.
	// For example channels[1][10] will point to the 11th sample in the 2nd
	// channel
	float** const	channels;



	int32_t			reserved[20];
};



/***** FUNCTION CALL ORDER DURING INITIALIZATION ******/
/*
    When the TOP loads the dll the functions will be called in this order

    setupParameters(OP_ParameterManager* m);

*/

/***** FUNCTION CALL ORDER DURING A COOK ******/
/*

	When the CHOP cooks the functions will be called in this order

	getGeneralInfo()
	getOutputInfo()
	if getOutputInfo() returns true
	{
		getChannelName() once for each channel needed 
	}
	execute()
	getNumInfoCHOPChans()
	for the number of chans returned getNumInfoCHOPChans()
	{
		getInfoCHOPChan()
	}
	getInfoDATSize()
	for the number of rows/cols returned by getInfoDATSize()
	{
		getInfoDATEntries()
	}
	getInfoPopupString()
	getWarningString()
	getErrorString()
*/

/*** DO NOT EDIT THIS CLASS, MAKE A SUBCLASS OF IT INSTEAD ***/
class CHOP_CPlusPlusBase
{
protected:
	CHOP_CPlusPlusBase()
	{
	}

	virtual ~CHOP_CPlusPlusBase()
	{
	}

public:


	// BEGIN PUBLIC INTERFACE

	// Some general settings can be assigned here (if you override it)
	virtual void
	getGeneralInfo(CHOP_GeneralInfo*, const OP_Inputs *inputs, void* reserved1)
	{
	}


	// This function is called so the class can tell the CHOP how many
	// channels it wants to output, how many samples etc.
	// Return true if you specify the output here.
	// Return false if you want the output to be set by matching
	// the channel names, numSamples, sample rate etc. of one of your inputs
	// The input that is used is chosen by setting the 'inputMatchIndex'
	// memeber in CHOP_OutputInfo
	// The CHOP_OutputInfo class is pre-filled with what the CHOP would
	// output if you return false, so you can just tweak a few settings
	// and return true if you want
	virtual bool		
	getOutputInfo(CHOP_OutputInfo*, const OP_Inputs *inputs, void *reserved1)
	{
		return false;
	}


	// This function will be called after getOutputInfo() asking for
	// the channel names. It will get called once for each channel name
	// you need to specify. If you returned 'false' from getOutputInfo()
	// it won't be called.
	virtual void
	getChannelName(int32_t index, OP_String *name,
					const OP_Inputs *inputs, void* reserved1)
	{
		name->setString("chan1");
	}


	// In this function you do whatever you want to fill the output channels
	// which are already allocated for you in 'outputs'
	virtual void		execute(CHOP_Output* outputs,
								const OP_Inputs* inputs,
								void* reserved1) = 0;


	// Override these methods if you want to output values to the Info CHOP/DAT
	// returning 0 means you dont plan to output any Info CHOP channels
	virtual int32_t		
	getNumInfoCHOPChans(void *reserved1)
	{
		return 0;
	}

	// Specify the name and value for Info CHOP channel 'index',
	// by assigning something to 'name' and 'value' members of the
	// OP_InfoCHOPChan class pointer that is passed in.
	virtual void
	getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void* reserved1)
	{
	}


	// Return false if you arn't returning data for an Info DAT
	// Return true if you are.
	// Set the members of the CHOP_InfoDATSize class to specify
	// the dimensions of the Info DAT
	virtual bool		
	getInfoDATSize(OP_InfoDATSize* infoSize, void *reserved1)
	{
		return false;
	}

	// You are asked to assign values to the Info DAT 1 row or column at a time
	// The 'byColumn' variable in 'getInfoDATSize' is how you specify
	// if it is by column or by row.
	// 'index' is the row/column index
	// 'nEntries' is the number of entries in the row/column
	// Strings should be UTF-8 encoded.
	virtual void	
	getInfoDATEntries(int32_t index, int32_t nEntries,
										OP_InfoDATEntries* entries,
										void *reserved1)
	{
	}

	// You can use this function to put the node into a warning state
	// by calling setString() on 'warning' with a non empty string.
	// Leave 'warning' unchanged to not go into warning state.
	virtual void
	getWarningString(OP_String *warning, void *reserved1) 
	{
	}

	// You can use this function to put the node into a error state
	// by calling setString() on 'error' with a non empty string.
	// Leave 'error' unchanged to not go into error state.
	virtual void
	getErrorString(OP_String *error, void *reserved1) 
	{
	}

	// Use this function to return some text that will show up in the
	// info popup (when you middle click on a node)
	// call setString() on info and give it some info if desired.
	virtual void
	getInfoPopupString(OP_String *info, void *reserved1) 
	{
	}


	// Override these methods if you want to define specfic parameters
	virtual void        
	setupParameters(OP_ParameterManager* manager, void* reserved1)
	{
	}


	// This is called whenever a pulse parameter is pressed
	virtual void
	pulsePressed(const char* name, void* reserved1)
	{
	}

	// END PUBLIC INTERFACE
				

private:

	// Reserved for future features
	virtual int32_t	reservedFunc6() { return 0; }
	virtual int32_t	reservedFunc7() { return 0; }
	virtual int32_t	reservedFunc8() { return 0; }
	virtual int32_t	reservedFunc9() { return 0; }
	virtual int32_t	reservedFunc10() { return 0; }
	virtual int32_t	reservedFunc11() { return 0; }
	virtual int32_t	reservedFunc12() { return 0; }
	virtual int32_t	reservedFunc13() { return 0; }
	virtual int32_t	reservedFunc14() { return 0; }
	virtual int32_t	reservedFunc15() { return 0; }
	virtual int32_t	reservedFunc16() { return 0; }
	virtual int32_t	reservedFunc17() { return 0; }
	virtual int32_t	reservedFunc18() { return 0; }
	virtual int32_t	reservedFunc19() { return 0; }
	virtual int32_t	reservedFunc20() { return 0; }

	int32_t			reserved[400];

};

static_assert(offsetof(CHOP_PluginInfo, apiVersion) == 0, "Incorrect Alignment");
static_assert(offsetof(CHOP_PluginInfo, customOPInfo) == 408, "Incorrect Alignment");
static_assert(sizeof(CHOP_PluginInfo) == 944, "Incorrect Size");

static_assert(offsetof(CHOP_GeneralInfo, cookEveryFrame) == 0, "Incorrect Alignment");
static_assert(offsetof(CHOP_GeneralInfo, cookEveryFrameIfAsked) == 1, "Incorrect Alignment");
static_assert(offsetof(CHOP_GeneralInfo, timeslice) == 2, "Incorrect Alignment");
static_assert(offsetof(CHOP_GeneralInfo, inputMatchIndex) == 4, "Incorrect Alignment");
static_assert(sizeof(CHOP_GeneralInfo) == 88, "Incorrect Size");

static_assert(offsetof(CHOP_OutputInfo, numChannels) == 0, "Incorrect Alignment");
static_assert(offsetof(CHOP_OutputInfo, numSamples) == 4, "Incorrect Alignment");
static_assert(offsetof(CHOP_OutputInfo, startIndex) == 8, "Incorrect Alignment");
static_assert(offsetof(CHOP_OutputInfo, sampleRate) == 12, "Incorrect Alignment");
static_assert(offsetof(CHOP_OutputInfo, reserved1) == 16, "Incorrect Alignment");
static_assert(sizeof(CHOP_OutputInfo) == 104, "Incorrect Size");

static_assert(offsetof(CHOP_Output, numChannels) == 0, "Incorrect Alignment");
static_assert(offsetof(CHOP_Output, numSamples) == 4, "Incorrect Alignment");
static_assert(offsetof(CHOP_Output, sampleRate) == 8, "Incorrect Alignment");
static_assert(offsetof(CHOP_Output, startIndex) == 12, "Incorrect Alignment");
static_assert(offsetof(CHOP_Output, names) == 16, "Incorrect Alignment");
static_assert(offsetof(CHOP_Output, channels) == 24, "Incorrect Alignment");
static_assert(sizeof(CHOP_Output) == 112, "Incorrect Size");
#endif
