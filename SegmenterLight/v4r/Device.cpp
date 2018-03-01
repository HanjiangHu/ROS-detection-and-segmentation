/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/


// --------------------------------
// Includes
// --------------------------------
#include "Device.h"
// #include "Draw.h"
#include <math.h>




namespace kinect
{
// --------------------------------
// Defines
// --------------------------------
#define MAX_STRINGS 20

// --------------------------------
// Global Variables
// --------------------------------
xn::Context g_Context;

DeviceStringProperty g_PrimaryStream;
DeviceParameter g_Registration;
DeviceParameter g_Resolution;
bool g_bIsDepthOn = true;
bool g_bIsImageOn = true;
bool g_bIsIROn = true;
bool g_bIsAudioOn = true;
bool g_bIsPlayerOn = true;

Device g_Device;
DepthGenerator g_Depth;
ImageGenerator g_Image;
IRGenerator g_IR;
AudioGenerator g_Audio;
Player g_Player;

DepthMetaData g_DepthMD;
ImageMetaData g_ImageMD;
IRMetaData g_irMD;
AudioMetaData g_AudioMD;

ProductionNode* g_pPrimary = NULL;

// --------------------------------
// Code
// --------------------------------
void initConstants()
{
	// Primary Streams
	int nIndex = 0;

	g_PrimaryStream.pValues[nIndex++] = "Any";
	g_PrimaryStream.pValues[nIndex++] = xnProductionNodeTypeToString(XN_NODE_TYPE_DEPTH);
	g_PrimaryStream.pValues[nIndex++] = xnProductionNodeTypeToString(XN_NODE_TYPE_IMAGE);
	g_PrimaryStream.pValues[nIndex++] = xnProductionNodeTypeToString(XN_NODE_TYPE_IR);
	g_PrimaryStream.pValues[nIndex++] = xnProductionNodeTypeToString(XN_NODE_TYPE_AUDIO);

	g_PrimaryStream.nValuesCount = nIndex;

	// Registration
	nIndex = 0;

	g_Registration.pValues[nIndex++] = FALSE;
	g_Registration.pValueToName[FALSE] = "Off";

	g_Registration.pValues[nIndex++] = TRUE;
	g_Registration.pValueToName[TRUE] = "Depth -> Image";

	g_Registration.nValuesCount = nIndex;

	// Resolutions
	nIndex = 0;

	g_Resolution.pValues[nIndex++] = XN_RES_QVGA;
	g_Resolution.pValueToName[XN_RES_QVGA] = Resolution(XN_RES_QVGA).GetName();

	g_Resolution.pValues[nIndex++] = XN_RES_VGA;
	g_Resolution.pValueToName[XN_RES_VGA] = Resolution(XN_RES_VGA).GetName();

	g_Resolution.pValues[nIndex++] = XN_RES_SXGA;
	g_Resolution.pValueToName[XN_RES_SXGA] = Resolution(XN_RES_SXGA).GetName();

	g_Resolution.pValues[nIndex++] = XN_RES_UXGA;
	g_Resolution.pValueToName[XN_RES_UXGA] = Resolution(XN_RES_UXGA).GetName();

	g_Resolution.nValuesCount = nIndex;
}

void XN_CALLBACK_TYPE onErrorStateChanged(XnStatus errorState, void* pCookie)
{
  printf("Kinect::Device::onErrorStateChanged: Warning: Not yet implemented!\n");
// 	if (errorState != XN_STATUS_OK)
// 	{
// 		setErrorState(xnGetStatusString(errorState));
// 	}
// 	else
// 	{
// 		setErrorState(NULL);
// 	}
}

void openCommon()
{
	XnStatus nRetVal = XN_STATUS_OK;

	g_bIsDepthOn = false;
	g_bIsImageOn = false;
	g_bIsIROn = false;
	g_bIsAudioOn = false;

	NodeInfoList list;
	nRetVal = g_Context.EnumerateExistingNodes(list);
	if (nRetVal == XN_STATUS_OK)
	{
		for (NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it)
		{
			switch ((*it).GetDescription().Type)
			{
			case XN_NODE_TYPE_DEVICE:
				(*it).GetInstance(g_Device);
				break;
			case XN_NODE_TYPE_DEPTH:
				g_bIsDepthOn = true;
				(*it).GetInstance(g_Depth);
				break;
			case XN_NODE_TYPE_IMAGE:
				g_bIsImageOn = true;
				(*it).GetInstance(g_Image);
				break;
			case XN_NODE_TYPE_IR:
				g_bIsIROn = true;
				(*it).GetInstance(g_IR);
				break;
			case XN_NODE_TYPE_AUDIO:
				g_bIsAudioOn = true;
				(*it).GetInstance(g_Audio);
				break;
			case XN_NODE_TYPE_PLAYER:
				g_bIsPlayerOn = true;
				(*it).GetInstance(g_Player);
			}
		}
	}

	XnCallbackHandle hDummy;
	g_Context.RegisterToErrorStateChange(onErrorStateChanged, NULL, hDummy);

	initConstants();

	readFrame();
}

XnStatus openDeviceFile(const char* csFile)
{
	XnStatus nRetVal = g_Context.Init();
	XN_IS_STATUS_OK(nRetVal);
	nRetVal = g_Context.OpenFileRecording(csFile);
	XN_IS_STATUS_OK(nRetVal);
	openCommon();

	return XN_STATUS_OK;
}

XnStatus openDeviceFromXml(const char* csXmlFile, EnumerationErrors& errors)
{
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = g_Context.InitFromXmlFile(csXmlFile, &errors);
	XN_IS_STATUS_OK(nRetVal);

	openCommon();

	return (XN_STATUS_OK);
}

void closeDevice()
{
	g_Context.Shutdown();
}

void readFrame()
{
	XnStatus rc = XN_STATUS_OK;

	if (g_pPrimary != NULL)
	{
		rc = g_Context.WaitOneUpdateAll(*g_pPrimary);
	}
	else
	{
		rc = g_Context.WaitAnyUpdateAll();
	}

	if (rc != XN_STATUS_OK)
	{
		printf("Error: %s\n", xnGetStatusString(rc));
	}

	if (g_Depth.IsValid())
	{
		g_Depth.GetMetaData(g_DepthMD);
	}

	if (g_Image.IsValid())
	{
		g_Image.GetMetaData(g_ImageMD);
	}

	if (g_IR.IsValid())
	{
		g_IR.GetMetaData(g_irMD);
	}

	if (g_Audio.IsValid())
	{
		g_Audio.GetMetaData(g_AudioMD);
	}
}

void changeRegistration(int nValue)
{
	if (!g_Depth.IsValid() || !g_Depth.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
	{
		return;
	}

	if (!nValue)
	{
		g_Depth.GetAlternativeViewPointCap().ResetViewPoint();
	}
	else if (g_Image.IsValid())
	{
		g_Depth.GetAlternativeViewPointCap().SetViewPoint(g_Image);
	}
}

void changePrimaryStream(int nIndex)
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (nIndex == 0)
	{
		g_pPrimary = NULL;
		return;
	}

	XnProductionNodeType Type;
	xnProductionNodeTypeFromString(g_PrimaryStream.pValues[nIndex], &Type);

	switch (Type)
	{
	case XN_NODE_TYPE_DEPTH:
		g_pPrimary = getDepthGenerator();
		break;
	case XN_NODE_TYPE_IMAGE:
		g_pPrimary = getImageGenerator();
		break;
	case XN_NODE_TYPE_IR:
		g_pPrimary = getIRGenerator();
		break;
	case XN_NODE_TYPE_AUDIO:
		g_pPrimary = getAudioGenerator();
		break;
	}
}

void createStream(Generator& generator, XnProductionNodeType type)
{
	EnumerationErrors errors;
	XnStatus nRetVal = g_Context.CreateAnyProductionTree(type, NULL, generator, &errors);
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
// 		displayMessage("%s", strError);
printf("Kinect::Device::createStream: Error: %s\n", strError);
		return;
	}
	else if (nRetVal != XN_STATUS_OK)
	{
// 		displayMessage("Open failed: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::createStream: Error: Open failed: %s\n", xnGetStatusString(nRetVal));
		return;
	}
}

void toggleStream(Generator& generator, XnProductionNodeType type, bool* bIsOn)
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (!generator.IsValid())
	{
		createStream(generator, type);
	}

	if (!generator.IsValid())
	{
		// failed creating the stream
		return;
	}

	if (generator.IsGenerating())
	{
		generator.StopGenerating();
	}
	else
	{
		generator.StartGenerating();
		if (nRetVal != XN_STATUS_OK)
		{
// 			displayMessage("Failed to turn on %s: %s", generator.GetInfo().GetInstanceName(), xnGetStatusString(nRetVal));
printf("Kinect::Device::toggleStream: Error: Failed to turn on %s: %s\n", generator.GetInfo().GetInstanceName(), xnGetStatusString(nRetVal));
			return;
		}
	}

	*bIsOn = (generator.IsGenerating() == TRUE);
}

void toggleDepthState(int nDummy)
{
	toggleStream(g_Depth, XN_NODE_TYPE_DEPTH, &g_bIsDepthOn);
}

void toggleImageState(int nDummy)
{
	toggleStream(g_Image, XN_NODE_TYPE_IMAGE, &g_bIsImageOn);
}

void toggleIRState(int nDummy)
{
	toggleStream(g_IR, XN_NODE_TYPE_IR, &g_bIsIROn);
}

void toggleAudioState(int nDummy)
{
	toggleStream(g_Audio, XN_NODE_TYPE_AUDIO, &g_bIsAudioOn);
}

void toggleMirror(int nDummy)
{
	XnStatus nRetVal = g_Context.SetGlobalMirror(!g_Context.GetGlobalMirror());
	if (nRetVal != XN_STATUS_OK)
	{
// 		displayMessage("Failed to set global mirror: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::toggleMirror: Error: Failed to set global mirror: %s\n", xnGetStatusString(nRetVal));
	}
}

void seekFrame(int nDiff)
{
	XnStatus nRetVal = XN_STATUS_OK;
	if (isPlayerOn())
	{
		const XnChar* strNodeName = NULL;
		if (g_pPrimary != NULL)
		{
			strNodeName = g_pPrimary->GetName();
		}
		else if (g_Depth.IsValid())
		{
			strNodeName = g_Depth.GetName();
		}
		else if (g_Image.IsValid())
		{
			strNodeName = g_Image.GetName();
		}
		else if (g_IR.IsValid())
		{
			strNodeName = g_IR.GetName();
		}
		else if (g_Audio.IsValid())
		{
			strNodeName = g_Audio.GetName();
		}

		nRetVal = g_Player.SeekToFrame(strNodeName, nDiff, XN_PLAYER_SEEK_CUR);
		if (nRetVal != XN_STATUS_OK)
		{
// 			displayMessage("Failed to seek: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::seekFrame: Error: Failed to seek: %s\n", xnGetStatusString(nRetVal));
			return;
		}

		XnUInt32 nFrame = 0;
		XnUInt32 nNumFrames = 0;
		nRetVal = g_Player.TellFrame(strNodeName, nFrame);
		if (nRetVal != XN_STATUS_OK)
		{
// 			displayMessage("Failed to tell frame: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::seekFrame: Error: Failed to tell frame: %s\n", xnGetStatusString(nRetVal));
			return;
		}

		nRetVal = g_Player.GetNumFrames(strNodeName, nNumFrames);
		if (nRetVal != XN_STATUS_OK)
		{
// 			displayMessage("Failed to get number of frames: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::seekFrame: Error: Failed to get number of frames: %s\n", xnGetStatusString(nRetVal));
			return;
		}

// 		displayMessage("Seeked %s to frame %u/%u", strNodeName, nFrame, nNumFrames);
printf("Kinect::Device::seekFrame: Error: Seeked %s to frame %u/%u\n", strNodeName, nFrame, nNumFrames);
	}	
}

bool isDepthOn()
{
	return (g_bIsDepthOn);
}

bool isImageOn()
{
	return (g_bIsImageOn);
}

bool isIROn()
{
	return (g_bIsIROn);
}

bool isAudioOn()
{
	return (g_bIsAudioOn);
}

bool isPlayerOn()
{
	return (g_bIsPlayerOn);
}

void setResolution(MapGenerator* pGenerator, int res)
{
printf("Kinect::Device::setResolution: %u\n", res);

	if (pGenerator == NULL)
	{
// 		displayMessage("Stream does not exist!");
printf("Kinect::Device::setResolution: Error: Stream does not exist!\n");
		return;
	}

	XnMapOutputMode Mode;
	pGenerator->GetMapOutputMode(Mode);
	Mode.nXRes = Resolution((XnResolution)res).GetXResolution();
	Mode.nYRes = Resolution((XnResolution)res).GetYResolution();
	XnStatus nRetVal = pGenerator->SetMapOutputMode(Mode);
	if (nRetVal != XN_STATUS_OK)
	{
// 		displayMessage("Failed to set resolution: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::setResolution: Error: Failed to set resolution: %s\n", xnGetStatusString(nRetVal));
	}
}

void setFPS(MapGenerator* pGenerator, int fps)
{
printf("Kinect::Device::setFPS: %u\n", fps);

	if (pGenerator == NULL)
	{
// 		displayMessage("Stream does not exist!");
printf("Kinect::Device::setFPS: Error: Stream does not exist!\n");
		return;
	}

	XnMapOutputMode Mode;
	pGenerator->GetMapOutputMode(Mode);
	Mode.nFPS = fps;
	XnStatus nRetVal = pGenerator->SetMapOutputMode(Mode);
	if (nRetVal != XN_STATUS_OK)
	{
// 		displayMessage("Failed to set FPS: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::setFPS: Error: Failed to set FPS: %s\n",  xnGetStatusString(nRetVal));
	}
}

void setDepthResolution(int res)
{
	setResolution(getDepthGenerator(), res);
}

void setDepthFPS(int fps)
{
	setFPS(getDepthGenerator(), fps);
}

void setImageResolution(int res)
{
	setResolution(getImageGenerator(), res);
}

void setImageFPS(int fps)
{
	setFPS(getImageGenerator(), fps);
}

void setIRResolution(int res)
{
	setResolution(getIRGenerator(), res);
}

void setIRFPS(int fps)
{
	setFPS(getIRGenerator(), fps);
}

void setStreamCropping(MapGenerator* pGenerator, XnCropping* pCropping)
{
	if (pGenerator == NULL)
	{
// 		displayMessage("Stream does not exist!");
printf("Kinect::Device::setStreamCropping: Error: Stream does not exist!\n");
		return;
	}

	if (!pGenerator->IsCapabilitySupported(XN_CAPABILITY_CROPPING))
	{
// 		displayMessage("Stream does not support cropping!");
printf("Kinect::Device::setStreamCropping: Error: Stream does not support cropping!\n");
		return;
	}

	XnStatus nRetVal = pGenerator->GetCroppingCap().SetCropping(*pCropping);
	if (nRetVal != XN_STATUS_OK)
	{
// 		displayMessage("Failed to set cropping: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::setStreamCropping: Error: Failed to set cropping: %s\n",  xnGetStatusString(nRetVal));
		return;
	}
}

void setPlaybackSpeed(int ratioDiff)
{
	if (g_Player.IsValid())
	{
		XnDouble dNewSpeed = g_Player.GetPlaybackSpeed() * pow(2.0, (XnDouble)ratioDiff);
		XnStatus nRetVal = g_Player.SetPlaybackSpeed(dNewSpeed);
		if (nRetVal != XN_STATUS_OK)
		{
// 			displayMessage("Failed to set playback speed: %s", xnGetStatusString(nRetVal));
printf("Kinect::Device::setPlaybackSpeed: Error: Failed to set playback speed: %s\n",  xnGetStatusString(nRetVal));
		}
	}
	else
	{
// 		displayMessage("Can't set playback speed - input is not a recording!");
printf("Kinect::Device::setPlaybackSpeed: Error: Can't set playback speed - input is not a recording!\n");
	}
}

XnDouble getPlaybackSpeed()
{
	if (g_Player.IsValid())
	{
		return g_Player.GetPlaybackSpeed();
	}
	else
	{
		return 1.0;
	}
}

Device* getDevice()
{
	return g_Device.IsValid() ? &g_Device : NULL;
}
DepthGenerator* getDepthGenerator()
{
	return g_Depth.IsValid() ? &g_Depth : NULL;
}
ImageGenerator* getImageGenerator()
{
	return g_Image.IsValid() ? &g_Image : NULL;
}
IRGenerator* getIRGenerator()
{
	return g_IR.IsValid() ? &g_IR : NULL;
}
AudioGenerator* getAudioGenerator()
{
	return g_Audio.IsValid() ? &g_Audio : NULL;
}

const DepthMetaData* getDepthMetaData()
{
	return g_Depth.IsValid() ? &g_DepthMD : NULL;
}
const ImageMetaData* getImageMetaData()
{
	return g_Image.IsValid() ? &g_ImageMD : NULL;
}
const IRMetaData* getIRMetaData()
{
	return g_IR.IsValid() ? &g_irMD : NULL;
}
const AudioMetaData* getAudioMetaData()
{
	return g_Audio.IsValid() ? &g_AudioMD : NULL;
}

}