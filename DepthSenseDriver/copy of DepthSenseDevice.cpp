/*****************************************************************************
The MIT License (MIT)

Copyright (c) 2013 Kaoru NAKAMURA, Tomoto WASHIO

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

This file is derived from DummyDevice.cpp provided by OpenNI 2.0.
*****************************************************************************/

#include "Driver/OniDriverAPI.h"
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"
#include "XnPlatform.h"

#include <DepthSense.hxx>
#include "../shared/ConversionTools.hxx"
#include <algorithm>
#include <map>

#define COLOR_WIDTH 640
#define COLOR_HEIGHT 480
#define COLOR_NPIXELS 307200
#define COLOR_DEFAULT_COMPONENT 255
#define DEPTH_ACQUIRED_WIDTH 320
#define DEPTH_ACQUIRED_HEIGHT 240
#define DEPTH_ACQUIRED_NPIXELS 76800
#define DEPTH_INTERPOLATED_WIDTH 640
#define DEPTH_INTERPOLATED_HEIGHT 480
#define DEPTH_INTERPOLATED_NPIXELS 307200
#define INVALID_DEPTH_VALUE 0
#define INVALID_DEPTH_CONFIDENCE_THRESHOLD 50

namespace DepthSenseGlobal {
    uint16_t pixelsDepthAcqVGA[DEPTH_INTERPOLATED_NPIXELS];
    uint16_t pixelsConfidenceVGA[DEPTH_INTERPOLATED_NPIXELS];
    uint8_t pixelsColorAcqVGA[3*COLOR_NPIXELS];
    uint8_t pixelsColorSyncVGA[3*COLOR_NPIXELS];
    DepthSense::UV uvMapVGA[COLOR_NPIXELS];
    int colorPixelCol, colorPixelRow, colorPixelInd;
}

class DepthSenseStream : public oni::driver::StreamBase
{
public:
	~DepthSenseStream()
	{
		stop();
	}

	xnl::OSEvent m_osEvent;

	OniStatus start()
	{
		m_osEvent.Create(TRUE);

		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	void stop()
	{
		m_running = false;
	}

	virtual OniStatus SetVideoMode(OniVideoMode*) = 0;
	virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode) = 0;

	OniStatus getProperty(int propertyId, void* data, int* pDataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (*pDataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", *pDataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return GetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	OniStatus setProperty(int propertyId, const void* data, int dataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (dataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", dataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return SetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	virtual void Mainloop() = 0;
protected:
	// Thread
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		DepthSenseStream* pStream = (DepthSenseStream*)pThreadParam;
		pStream->m_running = true;
		pStream->Mainloop();

		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}


	int singleRes(int x, int y) {
		OniVideoMode mode;
		GetVideoMode( &mode );

		return y * mode.resolutionX + x;
	}

	bool m_running;

	XN_THREAD_HANDLE m_threadHandle;

};

class DepthSenseDepthStream : public DepthSenseStream
{
public:

	DepthSenseDepthStream( DepthSense::Context& context, DepthSense::Node node )
		: m_context( context )
	{
		m_depthNode = node.as<DepthSense::DepthNode>();
		configureDepthNode();
		m_context.registerNode( node );

		m_nodeMap[m_depthNode] = this;
	}

	void configureDepthNode()
	{
		m_depthNode.newSampleReceivedEvent().connect(&onNewDepthSample);

		DepthSense::DepthNode::Configuration config = m_depthNode.getConfiguration();
		config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
		config.framerate = 30;
		config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
		config.saturation = true;

		//m_depthNode.setEnableVertices(true);
        m_depthNode.setEnableUvMap(true);
		m_depthNode.setEnableDepthMap( true );
		//m_depthNode.setEnableAccelerometer( true );
        m_depthNode.setEnableConfidenceMap(true);

		try  {
			m_context.requestControl(m_depthNode,0);

			m_depthNode.setConfiguration(config);
		}
		catch (std::exception& e) {
			fprintf( stderr, "exception : %s\n", e.what() );
		}

		//int32_t w, h;
		//DepthSense::FrameFormat_toResolution( config.frameFormat, &w, &h );
		//m_data.resize( w * h );
		m_data.resize( DEPTH_INTERPOLATED_NPIXELS );
	}


	static void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
	{
		DepthSenseDepthStream* pStream = m_nodeMap[node];
		if ( pStream ) {
			//fprintf( stderr, "onNewDepthSample\n" );
            rescaleMap(data.confidenceMap, DepthSenseGlobal::pixelsConfidenceVGA, DEPTH_ACQUIRED_WIDTH, DEPTH_ACQUIRED_HEIGHT, DEPTH_INTERPOLATED_WIDTH, DEPTH_INTERPOLATED_HEIGHT);
            rescaleMap(data.depthMap, DepthSenseGlobal::pixelsDepthAcqVGA, DEPTH_ACQUIRED_WIDTH, DEPTH_ACQUIRED_HEIGHT, DEPTH_INTERPOLATED_WIDTH, DEPTH_INTERPOLATED_HEIGHT, DepthSenseGlobal::pixelsConfidenceVGA, INVALID_DEPTH_CONFIDENCE_THRESHOLD, INVALID_DEPTH_VALUE);
            rescaleMap(data.uvMap, DepthSenseGlobal::uvMapVGA, DEPTH_ACQUIRED_WIDTH, DEPTH_ACQUIRED_HEIGHT, DEPTH_INTERPOLATED_WIDTH, DEPTH_INTERPOLATED_HEIGHT);
			xnOSMemCopy( &pStream->m_data[0], DepthSenseGlobal::pixelsDepthAcqVGA,  pStream->m_data.size() * 2 );


			pStream->m_osEvent.Set();
		}
		else {
			fprintf( stderr, "onNewDepthSample : no node pointer" );
		}
	}

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		pVideoMode->fps = 30;
		pVideoMode->resolutionX = DEPTH_INTERPOLATED_WIDTH;
		pVideoMode->resolutionY = DEPTH_INTERPOLATED_HEIGHT;
		return ONI_STATUS_OK;
	}

private:

	void Mainloop()
	{
		int frameId = 1;
		while (m_running)
		{
			m_osEvent.Wait(XN_WAIT_INFINITE);
			m_osEvent.Reset();

			OniFrame* pFrame = getServices().acquireFrame();
			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Fill frame
			xnOSMemSet(pFrame->data, 0, pFrame->dataSize);

			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
			pFrame->videoMode.resolutionX = DEPTH_INTERPOLATED_WIDTH;
			pFrame->videoMode.resolutionY = DEPTH_INTERPOLATED_HEIGHT;
			pFrame->videoMode.fps = 30;

			pFrame->width = DEPTH_INTERPOLATED_WIDTH;
			pFrame->height = DEPTH_INTERPOLATED_HEIGHT;

			xnOSMemCopy( pFrame->data, &m_data[0], m_data.size() * 2 );

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_DEPTH;
			pFrame->stride = DEPTH_INTERPOLATED_WIDTH*sizeof(OniDepthPixel);
			pFrame->timestamp = frameId * 33000;

			raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);

			frameId++;
		}
	}

	DepthSense::Context& m_context;
	DepthSense::DepthNode m_depthNode;

	std::vector<int16_t> m_data;
	static std::map<DepthSense::DepthNode, DepthSenseDepthStream*> m_nodeMap;
};

/*static*/ std::map<DepthSense::DepthNode, DepthSenseDepthStream*> DepthSenseDepthStream::m_nodeMap;


class DepthSenseImageStream : public DepthSenseStream
{
public:

	DepthSenseImageStream( DepthSense::Context& context, DepthSense::Node node )
		: m_context( context )
	{
		m_colorNode = node.as<DepthSense::ColorNode>();
        configureColorNode();
        m_context.registerNode(node);

		m_nodeMap[m_colorNode] = this;
	}

	// Create color stream of DepthSense SDK
	// DepthSense SDK のカラーストリームを作成
	void configureColorNode()
	{
		// connect new color sample handler
		m_colorNode.newSampleReceivedEvent().connect(&onNewColorSample);

		DepthSense::ColorNode::Configuration config = m_colorNode.getConfiguration();
		config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
		config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
		config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
		config.framerate = 30;

		m_colorNode.setEnableColorMap(true);

		try {
			m_context.requestControl( m_colorNode, 0 );

			m_colorNode.setConfiguration(config);

			//fprintf( stderr, "Image node config is success\n" );
		}
		catch (std::exception& e) {
			fprintf( stderr, "exception : %s\n", e.what() );
		}

		//int32_t w, h;
		//DepthSense::FrameFormat_toResolution( config.frameFormat, &w, &h );
		//m_data.resize( w * h * 3 );
		m_data.resize( COLOR_NPIXELS * 3 );
	}

	// Callback to update the frame of the color stream
	// カラーストリームのフレーム更新コールバック関数
	static void onNewColorSample( DepthSense::ColorNode node,
		                          DepthSense::ColorNode::NewSampleReceivedData data)
	{
		DepthSenseImageStream* pStream = m_nodeMap[node];
		if ( pStream ) {
			//unsigned char* d = &pStream->m_data[0];
			//const unsigned char* dend = d + pStream->m_data.size();
			//const unsigned char* s = data.colorMap;
            for (int currentPixelInd = 0; currentPixelInd < COLOR_NPIXELS; currentPixelInd++)
            {
                DepthSenseGlobal::pixelsColorAcqVGA[3*currentPixelInd] = data.colorMap[3*currentPixelInd+2];
                DepthSenseGlobal::pixelsColorAcqVGA[3*currentPixelInd+1] = data.colorMap[3*currentPixelInd+1];
                DepthSenseGlobal::pixelsColorAcqVGA[3*currentPixelInd+2] = data.colorMap[3*currentPixelInd];
                DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd] = COLOR_DEFAULT_COMPONENT;
                DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+1] = COLOR_DEFAULT_COMPONENT;
                DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+2] = COLOR_DEFAULT_COMPONENT;
            }

            for (int currentPixelInd = 0; currentPixelInd < COLOR_NPIXELS; currentPixelInd++)
            {
                uvToColorPixelInd(DepthSenseGlobal::uvMapVGA[currentPixelInd], COLOR_WIDTH, COLOR_HEIGHT, &DepthSenseGlobal::colorPixelInd, &DepthSenseGlobal::colorPixelRow, &DepthSenseGlobal::colorPixelCol);
                if (DepthSenseGlobal::colorPixelInd == -1) {
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd] = COLOR_DEFAULT_COMPONENT;
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+1] = COLOR_DEFAULT_COMPONENT;
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+2] = COLOR_DEFAULT_COMPONENT;
                }
                else
                {
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd] = DepthSenseGlobal::pixelsColorAcqVGA[3*DepthSenseGlobal::colorPixelInd];
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+1] = DepthSenseGlobal::pixelsColorAcqVGA[3*DepthSenseGlobal::colorPixelInd+1];
                    DepthSenseGlobal::pixelsColorSyncVGA[3*currentPixelInd+2] = DepthSenseGlobal::pixelsColorAcqVGA[3*DepthSenseGlobal::colorPixelInd+2];
                }
            }

			xnOSMemCopy( &pStream->m_data[0], DepthSenseGlobal::pixelsColorSyncVGA,  pStream->m_data.size() );

			pStream->m_osEvent.Set();
		}
		else {
			fprintf( stderr, "onNewColorSample : no node poiter\n" );
		}
	}

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pVideoMode->fps = 30;
		pVideoMode->resolutionX = COLOR_WIDTH;
		pVideoMode->resolutionY = COLOR_HEIGHT;
		return ONI_STATUS_OK;
	}

private:

	void Mainloop()
	{
		int frameId = 1;
		while (m_running)
		{
			m_osEvent.Wait(XN_WAIT_INFINITE);
			m_osEvent.Reset();

			OniFrame* pFrame = getServices().acquireFrame();
			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Fill frame
			xnOSMemSet(pFrame->data, 0, pFrame->dataSize);

			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
			pFrame->videoMode.resolutionX = COLOR_WIDTH;
			pFrame->videoMode.resolutionY = COLOR_HEIGHT;
			pFrame->videoMode.fps = 30;

			pFrame->width = COLOR_WIDTH;
			pFrame->height = COLOR_HEIGHT;

			xnOSMemCopy( pFrame->data, &m_data[0], m_data.size() );

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_COLOR;
			pFrame->stride = COLOR_WIDTH*sizeof(OniDepthPixel);
			pFrame->timestamp = frameId * 33000;

			raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);

			frameId++;
		}
	}

	DepthSense::Context& m_context;
	DepthSense::ColorNode m_colorNode;

	std::vector<unsigned char> m_data;
	static std::map<DepthSense::ColorNode, DepthSenseImageStream*> m_nodeMap;
};

/*static*/ std::map<DepthSense::ColorNode, DepthSenseImageStream*> DepthSenseImageStream::m_nodeMap;

class DepthSenseDevice : public oni::driver::DeviceBase
{
public:
	DepthSenseDevice( OniDeviceInfo* pInfo, oni::driver::DriverServices& driverServices,
		      DepthSense::Context& context, DepthSense::Device& device )
		: m_pInfo(pInfo)
		, m_driverServices(driverServices)
		, m_context( context )
		, m_device( device )
	{
		m_numSensors = 2;

		m_sensors[0].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
		m_sensors[0].numSupportedVideoModes = 1;
		m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		m_sensors[0].pSupportedVideoModes[0].fps = 30;
		//m_sensors[0].pSupportedVideoModes[0].resolutionX = DEPTH_INTERPOLATED_WIDTH;
		//m_sensors[0].pSupportedVideoModes[0].resolutionY = DEPTH_INTERPOLATED_HEIGHT;
		m_sensors[0].pSupportedVideoModes[0].resolutionX = 640;
		m_sensors[0].pSupportedVideoModes[0].resolutionY = 480;

		m_sensors[1].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[1].sensorType = ONI_SENSOR_COLOR;
		m_sensors[1].numSupportedVideoModes = 1;
		m_sensors[1].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		m_sensors[1].pSupportedVideoModes[0].fps = 30;
		m_sensors[1].pSupportedVideoModes[0].resolutionX = COLOR_WIDTH;
		m_sensors[1].pSupportedVideoModes[0].resolutionY = COLOR_HEIGHT;

	}

	OniDeviceInfo* GetInfo()
	{
		return m_pInfo;
	}

	OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
	{
		*numSensors = m_numSensors;
		*pSensors = m_sensors;

		return ONI_STATUS_OK;
	}

	oni::driver::StreamBase* createStream(OniSensorType sensorType)
	{
		if (sensorType == ONI_SENSOR_DEPTH)
		{
			// Find the depth node
			// Depth ノードを探す
	        std::vector<DepthSense::Node> nodes = m_device.getNodes();
			//auto it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
			std::vector<DepthSense::Node>::iterator it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
				return val.is<DepthSense::DepthNode>();
			} );

			// Create the depth stream
			// Depth ストリームを作成する
			if ( it != nodes.end() ) {
				DepthSenseDepthStream* pImage = XN_NEW( DepthSenseDepthStream, m_context, *it );
				return pImage;
			}
		}
		if (sensorType == ONI_SENSOR_COLOR)
		{
			// Find the color node
			// カラーノードを探す
	        std::vector<DepthSense::Node> nodes = m_device.getNodes();
			//auto it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
			std::vector<DepthSense::Node>::iterator it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
				return val.is<DepthSense::ColorNode>();
			} );

			// Create the color stream
			// カラーストリームを作成する
			if ( it != nodes.end() ) {
				DepthSenseImageStream* pImage = XN_NEW( DepthSenseImageStream, m_context, *it );
				return pImage;
			}
		}

		m_driverServices.errorLoggerAppend("DepthSenseDevice: Can't create a stream of type %d", sensorType);
		return NULL;
	}

	void destroyStream(oni::driver::StreamBase* pStream)
	{
		XN_DELETE(pStream);
	}

	OniStatus  getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus rc = ONI_STATUS_OK;

		switch (propertyId)
		{
		case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
			{
				if (*pDataSize == sizeof(OniVersion))
				{
					OniVersion* version = (OniVersion*)data;
					version->major = version->minor = version->maintenance = version->build = 2;
				}
				else
				{
					m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
					rc = ONI_STATUS_ERROR;
				}
			}
			break;
		default:
			m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
			rc = ONI_STATUS_ERROR;
		}
		return rc;
	}
private:
	DepthSenseDevice(const DepthSenseDevice&);
	void operator=(const DepthSenseDevice&);

	OniDeviceInfo* m_pInfo;
	int m_numSensors;
	OniSensorInfo m_sensors[10];
	oni::driver::DriverServices& m_driverServices;

	DepthSense::Context& m_context;
	DepthSense::Device m_device;
};


class DepthSenseDriver : public oni::driver::DriverBase
{
public:
	DepthSenseDriver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
	{}

	// Initialize the driver
	// ドライバを初期化する
	virtual OniStatus initialize(
		oni::driver::DeviceConnectedCallback connectedCallback,
		oni::driver::DeviceDisconnectedCallback disconnectedCallback,
		oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
		void* pCookie)
	{
printf("COUCOU SOFTKIN INIT \n");
		oni::driver::DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);

		// Connect to the DepthSense device
		// DepthSense デバイスに接続する
		m_context = DepthSense::Context::create("localhost");

		// Enumerate the connected devices
		// 接続されているデバイスの情報を作成する
		m_depthSenseDevices = m_context.getDevices();
		int deviceIndex = 0;
		for ( auto it = m_depthSenseDevices.begin(); it != m_depthSenseDevices.end(); ++it ) {
			// Populate the device information
			// デバイス情報をコピーする
			OniDeviceInfo* pInfo = XN_NEW(OniDeviceInfo);
			xnOSStrCopy(pInfo->vendor, "DepthSense", ONI_MAX_STR);
			xnOSStrCopy(pInfo->name, DepthSense::Device::Model_toString( it->getModel() ).c_str(), ONI_MAX_STR);

			XnUInt32 uriLen;
			xnOSStrFormat(pInfo->uri, ONI_MAX_STR, &uriLen, "%s/%d", pInfo->name, deviceIndex++);

			// Register the device information with the map
			// デバイス情報を登録する
			m_devices[pInfo] = NULL;

			// Raise events
			// デバイスの接続を通知する
			deviceConnected(pInfo);
			deviceStateChanged(pInfo, 0);
		}

		// Create the thread for main loop
		// デバイスのメインループ用スレッドを生成する
		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	virtual oni::driver::DeviceBase* deviceOpen(const char* uri, const char* /*mode*/)
	{
		for ( auto iter = m_devices.Begin(); iter != m_devices.End(); ++iter) {
			if (xnOSStrCmp(iter->Key()->uri, uri) == 0) {
				// Found
				if (iter->Value() != NULL) {
					// already using
					return iter->Value();
				}

				int deviceIndex = atoi(strrchr(uri, '/') + 1);

				// Create the device instance
				// デバイスインスタンスを生成する
				DepthSenseDevice* pDevice = XN_NEW(DepthSenseDevice, iter->Key(), getServices(), m_context, m_depthSenseDevices[deviceIndex] );
				iter->Value() = pDevice;
				return pDevice;
			}
            else printf("COUCOU\n");
		}

		getServices().errorLoggerAppend("Looking for '%s'", uri);
		return NULL;
	}

	virtual void deviceClose(oni::driver::DeviceBase* pDevice)
	{
		for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
		{
			if (iter->Value() == pDevice)
			{
				iter->Value() = NULL;
				XN_DELETE(pDevice);
				return;
			}
		}

		// not our device?!
		XN_ASSERT(FALSE);
	}

	void shutdown() {}

protected:

	// Thread for mail loop
	// デバイスのメインループ用スレッド
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		//fprintf( stderr, "context is running\n" );

		DepthSenseDriver* pDriver = (DepthSenseDriver*)pThreadParam;
		pDriver->m_context.startNodes();
		pDriver->m_context.run();
		pDriver->m_context.stopNodes();

		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}

	XN_THREAD_HANDLE m_threadHandle;

	xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*> m_devices;

	DepthSense::Context m_context;
	std::vector<DepthSense::Device> m_depthSenseDevices;
};

ONI_EXPORT_DRIVER(DepthSenseDriver);
