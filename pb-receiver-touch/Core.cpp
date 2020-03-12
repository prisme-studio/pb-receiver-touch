//
//  Core.cpp
//  pb-receiver-touch
//
//  Created by Valentin Dufois on 2019-11-19.
//

#include <pb-common/Structs/Body.hpp>

#include "Core.hpp"

Core::Core() {
	_receiver.open();
	_receiver.addObserver(this);
}

Core::~Core()
{
	_receiver.close();
}


// MARK: - TouchDesigner handles

void
Core::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
{
	// Set our parameters

	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->cookEveryFrame = true;

	// Note: To disable timeslicing you'll need to turn this off, as well as ensure that
	// getOutputInfo() returns true, and likely also set the info->numSamples to how many
	// samples you want to generate for this CHOP. Otherwise it'll take on length of the
	// input CHOP, which may be timesliced.
	ginfo->timeslice = false;
}

bool
Core::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
{
	// Since we are outputting a timeslice, the system will dictate
	// the numSamples and startIndex of the CHOP data
	info->numSamples = 1;
	info->startIndex = 0;

	// We are about to execute, get a copy of the bodies
	// Did we receive any bodies from the network ?

	_bodies = _receiver.arena()->getSubset();

	// Set the number of channels
	_outputPositions = inputs->getParInt("Pboutputpositions");
	_outputOrientations = inputs->getParInt("Pboutputorientations");
	_outputConfidences = inputs->getParInt("Pboutputconfs");

	info->numChannels = 1 + (int)_bodies.size() * getChannelCountByBody();

	return true;
}

void
Core::getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
{
	// First channel is the number of bodies
	if(index == 0) {
		name->setString("body_count");
		return;
	}

	int bodyChannels = getChannelCountByBody();
	int jointChannels = getChannelCountByJoint();

	int32_t bodyIndex = floor((index - 1) / bodyChannels);
	int32_t jointIndex = ((index - 1) % bodyChannels) / jointChannels;
	int32_t channelIndex = ((index - 1) % bodyChannels) % jointChannels;

	std::string channelName = "body" + std::to_string(getBodyIndex(_bodies[bodyIndex]->uid)) + "/" + getJointName(jointIndex) + ":" + getJointChannelName(channelIndex);

	 name->setString(channelName.c_str());
}

void
Core::execute(CHOP_Output* output, const OP_Inputs* inputs, void* reserved)
{
	// First, set the body count
	output->channels[0][0] = _bodies.size();
	unsigned int currChannel = 1;

	for(pb::Body * body: _bodies) {
		for(pb::Joint &joint: body->skeleton()->joints) {
			bool posConf = joint.positionConfidence > 0 && joint.positionConfidence <= 1.0;
			bool orConf = joint.orientationConfidence > 0 && joint.orientationConfidence <= 1.0;

			if(_outputPositions) {
				output->channels[currChannel + 0][0] = posConf ? joint.position.x : 0;
				output->channels[currChannel + 1][0] = posConf ? joint.position.y : 0;
				output->channels[currChannel + 2][0] = posConf ? -joint.position.z : 0;
				currChannel += 3;
			}

			if(_outputOrientations) {
				output->channels[currChannel + 0][0] = orConf ? joint.orientation.x : 0;
				output->channels[currChannel + 1][0] = orConf ? joint.orientation.y : 0;
				output->channels[currChannel + 2][0] = orConf ? joint.orientation.z : 0;
				currChannel += 3;
			}

			if(_outputConfidences) {
				output->channels[currChannel + 0][0] = joint.positionConfidence;
				output->channels[currChannel + 1][0] = joint.orientationConfidence;
				currChannel += 2;
			}
		}
	}
}

void
Core::setupParameters(OP_ParameterManager* manager, void *reserved1)
{
	OP_ParAppendResult res;

	// Output Selectors
	OP_NumericParameter positionsToggle;
	positionsToggle.name = "Pboutputpositions";
	positionsToggle.label = "Positions";
	positionsToggle.defaultValues[0] = 1;

	res = manager->appendToggle(positionsToggle);
	assert(res == OP_ParAppendResult::Success);

	OP_NumericParameter orientationsToggle;
	orientationsToggle.name = "Pboutputorientations";
	orientationsToggle.label = "Orientations";

	res = manager->appendToggle(orientationsToggle);
	assert(res == OP_ParAppendResult::Success);

	OP_NumericParameter confsToggle;
	confsToggle.name = "Pboutputconfs";
	confsToggle.label = "Confidences";

	res = manager->appendToggle(confsToggle);
	assert(res == OP_ParAppendResult::Success);

	// Body index reset
	OP_NumericParameter resetIndex;
	resetIndex.name = "Pbresetindexes";
	resetIndex.label = "Reset indexes";

	res = manager->appendPulse(resetIndex);
	assert(res == OP_ParAppendResult::Success);
}

void 
Core::pulsePressed(const char* name, void* reserved1)
{
}

void Core::getWarningString(OP_String * warning, void *reserved1) {
	if(!_isConnected) {
		warning->setString("Looking for a Locator Master on the network...");
	}
}

// MARK: - PBReceiverDelegate


void Core::receiverDidConnect(pb::PBReceiver *) {
	_isConnected = true;
};

void Core::receiverDidUpdate(pb::PBReceiver *) {};

void Core::receiverDidClose(pb::PBReceiver *) {
	_isConnected = false;
};


// MARK: - Internal

int Core::getChannelCountByBody() {
	return getChannelCountByJoint() * 15;
}

int Core::getChannelCountByJoint() {
	unsigned short int count = 0;

	if(_outputPositions)
		count += 3;

	if(_outputOrientations)
		count += 3;

	if(_outputConfidences)
		count += 2;

	return count;
}

unsigned long Core::getBodyIndex(const pb::bodyUID &bodyUID) {
	if(_bodiesIndex.find(bodyUID) != _bodiesIndex.end())
		return _bodiesIndex[bodyUID];

	_bodiesIndex[bodyUID] = _bodiesIndex.size();
	return _bodiesIndex[bodyUID];
}

std::string Core::getJointName(const int &jointIndex) {
	switch(jointIndex) {
		case  0: return "head";
		case  1: return "neck";
		case  2: return "leftShoulder";
		case  3: return "rightShoulder";
		case  4: return "leftElbow";
		case  5: return "rightElbow";
		case  6: return "leftHand";
		case  7: return "rightHand";
		case  8: return "torso";
		case  9: return "leftHip";
		case 10: return "rightHip";
		case 11: return "leftKnee";
		case 12: return "rightKnee";
		case 13: return "leftFoot";
		case 14: return "rightFoot";
		default: return std::to_string(jointIndex);
	}
}

std::string Core::getJointChannelName(const int &index) {

	int i = index;

	if(_outputPositions) {
		switch(i) {
			case  0: return "tx";
			case  1: return "ty";
			case  2: return "tz";
			default: i -= 3;
		}
	}

	if(_outputOrientations) {
		switch(i) {
			case  0: return "rx";
			case  1: return "ry";
			case  2: return "rz";
			case  4: return "rw";
			default: i -= 4;
		}
	}

	if(_outputConfidences) {
		switch(i) {
			case  0: return "tconf";
			case  1: return "rconf";
			default: i -= 2;
		}
	}

	return std::to_string(index);
}

