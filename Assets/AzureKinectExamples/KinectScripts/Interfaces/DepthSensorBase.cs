using Microsoft.Azure.Kinect.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace com.rfilkov.kinect
{
    public abstract class DepthSensorBase : MonoBehaviour, DepthSensorInterface
    {
        // max depth distance in mm, used for initializing data arrays and compute buffers
        public const int MAX_DEPTH_DISTANCE_MM = 10000;

        [Tooltip("Device streaming mode, in means of connected sensor, recording or disabled.")]
        public KinectInterop.DeviceStreamingMode deviceStreamingMode = KinectInterop.DeviceStreamingMode.ConnectedSensor;

        [Tooltip("Index of the depth sensor in the list of currently connected sensors.")]
        public int deviceIndex = 0;

        [Tooltip("Path to the recording file, if the streaming mode is PlayRecording.")]
        public string recordingFile = string.Empty;

        //[Tooltip("Sensor position in space.")]
        //public Vector3 devicePosition = new Vector3(0f, 1f, 0f);

        //[Tooltip("Sensor rotation in space.")]
        //public Vector3 deviceRotation = new Vector3(0f, 0f, 0f);

        //[Tooltip("Whether the body tracking for this sensor is enabled or not.")]
        //internal bool bodyTrackingEnabled = false;

        [Tooltip("Minimum distance in meters, used for creating the depth-related images.")]
        [Range(0f, 10f)]
        public float minDistance = 0.5f;

        [Tooltip("Maximum distance in meters, used for creating the depth-related images.")]
        [Range(0f, 10f)]
        public float maxDistance = 10f;

        [Tooltip("Resolution of the generated point-cloud textures.")]
        public PointCloudResolution pointCloudResolution = PointCloudResolution.DepthCameraResolution;
        public enum PointCloudResolution : int { DepthCameraResolution = 0, ColorCameraResolution = 1 }

        [Tooltip("Render texture, used for point-cloud vertex mapping. The texture resolution should match the depth or color image resolution.")]
        public RenderTexture pointCloudVertexTexture = null;

        [Tooltip("Render texture, used for point-cloud color mapping. The texture resolution should match the depth or color image resolution.")]
        public RenderTexture pointCloudColorTexture = null;


        // initial parameters
        protected KinectInterop.FrameSource frameSourceFlags;
        protected bool isSyncDepthAndColor = false;
        //protected bool isSyncBodyAndDepth = false;

        // initial pose parameters
        protected Vector3 initialPosePosition = Vector3.zero;
        protected Quaternion initialPoseRotation = Quaternion.identity;

        // frame numbers
        protected ulong colorFrameNumber = 0;
        protected ulong depthFrameNumber = 0;
        protected ulong infraredFrameNumber = 0;
        protected ulong poseFrameNumber = 0;

        // raw color data
        protected byte[] rawColorImage = null;
        protected ulong rawColorTimestamp = 0;
        protected ulong currentColorTimestamp = 0;
        protected object colorFrameLock = new object();

        // raw depth data
        protected ushort[] rawDepthImage = null;
        protected ulong rawDepthTimestamp = 0;
        protected ulong currentDepthTimestamp = 0;
        protected object depthFrameLock = new object();

        // raw infrared data
        protected ushort[] rawInfraredImage = null;
        protected ulong rawInfraredTimestamp = 0;
        protected ulong currentInfraredTimestamp = 0;
        protected object infraredFrameLock = new object();

        // raw pose data
        protected Vector3 rawPosePosition;
        protected Quaternion rawPoseRotation;
        protected ulong rawPoseTimestamp = 0;
        protected ulong currentPoseTimestamp = 0;
        protected object poseFrameLock = new object();

        // sensor pose data
        protected Vector3 sensorPosePosition;
        protected Quaternion sensorPoseRotation;


        // depth image data
        protected int[] depthHistBufferData = null;
        protected int[] equalHistBufferData = null;
        protected int histDataTotalPoints = 0;
        protected ulong lastDepthImageTimestamp = 0;
        protected object depthImageDataLock = new object();

        // last updated depth coord-frame time
        protected ulong lastDepthCoordFrameTime = 0;

        // point cloud vertex shader
        protected ComputeShader pointCloudVertexShader = null;
        protected int pointCloudVertexKernel = -1;
        protected Vector2Int pointCloudVertexRes = Vector2Int.zero;
        protected RenderTexture pointCloudVertexRT = null;
        protected ComputeBuffer pointCloudSpaceBuffer = null;
        protected ComputeBuffer pointCloudDepthBuffer = null;

        // point cloud color shader
        protected ComputeShader pointCloudColorShader = null;
        protected int pointCloudColorKernel = -1;
        protected Vector2Int pointCloudColorRes = Vector2Int.zero;
        protected RenderTexture pointCloudColorRT = null;
        protected ComputeBuffer pointCloudCoordBuffer = null;
        protected Texture2D pointCloudAlignedColorTex = null;

        // depth2color coords frame
        protected byte[] depth2ColorDataFrame = null;
        protected Vector2[] depth2ColorCoordFrame = null;
        protected ulong lastDepth2ColorFrameTime = 0;
        protected object depth2ColorFrameLock = new object();

        // color2depth coords frame
        protected ushort[] color2DepthDataFrame = null;
        protected Vector2[] color2DepthCoordFrame = null;
        protected ulong lastColor2DepthFrameTime = 0;
        protected object color2DepthFrameLock = new object();


        protected virtual void Awake()
        {
            // init raw sensor pose
            rawPosePosition = Vector3.zero;
            rawPoseRotation = Quaternion.identity;
            rawPoseTimestamp = (ulong)DateTime.Now.Ticks;

            sensorPosePosition = transform.position;
            sensorPoseRotation = transform.rotation;

            // initial pose params
            initialPosePosition = transform.position;
            initialPoseRotation = transform.rotation;
        }



        public abstract KinectInterop.DepthSensorPlatform GetSensorPlatform();

        public abstract List<KinectInterop.SensorDeviceInfo> GetAvailableSensors();

        public virtual KinectInterop.SensorData OpenSensor(KinectInterop.FrameSource dwFlags, bool bSyncDepthAndColor, bool bSyncBodyAndDepth)
        {
            // save the parameters for later
            frameSourceFlags = dwFlags;
            isSyncDepthAndColor = bSyncDepthAndColor && ((dwFlags & KinectInterop.FrameSource.TypeColor) != 0) && ((dwFlags & KinectInterop.FrameSource.TypeDepth) != 0);
            //isSyncBodyAndDepth = bSyncBodyAndDepth && ((dwFlags & KinectInterop.FrameSource.TypeBody) != 0) && ((dwFlags & KinectInterop.FrameSource.TypeDepth) != 0);

            return null;
        }


        public virtual void CloseSensor(KinectInterop.SensorData sensorData)
        {
            // dispose coord mapping shaders
            DisposePointCloudVertexShader(sensorData);
            DisposePointCloudColorShader(sensorData);
        }


        public virtual void InitSensorData(KinectInterop.SensorData sensorData, KinectManager kinectManager)
        {
            // depth image data
            if (kinectManager.getDepthFrames == KinectManager.DepthTextureType.DepthTexture)
            {
                depthHistBufferData = new int[MAX_DEPTH_DISTANCE_MM + 1];
                equalHistBufferData = new int[MAX_DEPTH_DISTANCE_MM + 1];
                sensorData.depthHistBufferData = new int[equalHistBufferData.Length];
            }
            else
            {
                depthHistBufferData = null;
                equalHistBufferData = null;
                sensorData.depthHistBufferData = null;
            }
        }

        public virtual void PollSensorFrames(KinectInterop.SensorData sensorData)
        {
        }


        public virtual void PollCoordTransformFrames(KinectInterop.SensorData sensorData)
        {
        }


        public virtual void ProcessSensorDataInThread(KinectInterop.SensorData sensorData)
        {
            // depth-image data
            if (lastDepthImageTimestamp != rawDepthTimestamp && rawDepthImage != null && depthHistBufferData != null)
            {
                lock (depthImageDataLock)
                {
                    Array.Clear(depthHistBufferData, 0, depthHistBufferData.Length);
                    Array.Clear(equalHistBufferData, 0, equalHistBufferData.Length);
                    histDataTotalPoints = 0;

                    int depthMinDistance = (int)(minDistance * 1000f);
                    int depthMaxDistance = (int)(maxDistance * 1000f);

                    for (int i = 0; i < rawDepthImage.Length; i++)
                    {
                        int depth = rawDepthImage[i];
                        int limDepth = (depth <= MAX_DEPTH_DISTANCE_MM) ? depth : 0;

                        if (limDepth > 0)
                        {
                            depthHistBufferData[limDepth]++;
                            histDataTotalPoints++;
                        }
                    }

                    equalHistBufferData[0] = depthHistBufferData[0];
                    for (int i = 1; i < depthHistBufferData.Length; i++)
                    {
                        equalHistBufferData[i] = equalHistBufferData[i - 1] + depthHistBufferData[i];
                    }

                    // make depth 0 equal to the max-depth
                    equalHistBufferData[0] = equalHistBufferData[equalHistBufferData.Length - 1];

                    lastDepthImageTimestamp = rawDepthTimestamp;
                    //Debug.Log("lastDepthImageTimestamp: " + lastDepthImageTimestamp);
                }
            }

            // ...


            // set the frame timestamps
            if (currentColorTimestamp != rawColorTimestamp)
            {
                // new color frame
                currentColorTimestamp = rawColorTimestamp;
            }

            if (currentDepthTimestamp != rawDepthTimestamp)
            {
                // new depth frame
                currentDepthTimestamp = rawDepthTimestamp;
            }

            if (currentInfraredTimestamp != rawInfraredTimestamp)
            {
                // new depth frame
                currentInfraredTimestamp = rawInfraredTimestamp;
            }

            if (currentPoseTimestamp != rawPoseTimestamp)
            {
                // new pose frame
                currentPoseTimestamp = rawPoseTimestamp;
            }
        }


        public virtual bool UpdateSensorData(KinectInterop.SensorData sensorData, KinectManager kinectManager)
        {
            // color frame
            lock (colorFrameLock)
            {
                if (rawColorImage != null && sensorData.lastColorFrameTime != currentColorTimestamp)
                {
                    Texture2D colorImageTex2D = sensorData.colorImageTexture as Texture2D;
                    if (colorImageTex2D != null)
                    {
                        colorImageTex2D.LoadRawTextureData(rawColorImage);
                        colorImageTex2D.Apply();
                    }

                    sensorData.lastColorFrameTime = currentColorTimestamp;
                    //Debug.Log("Color frame. Timestamp: " + currentColorTimestamp);
                }
            }

            // depth frame
            lock (depthFrameLock)
            {
                if (rawDepthImage != null && sensorData.lastDepthFrameTime != currentDepthTimestamp)
                {
                    // depth image
                    if (sensorData.depthImage != null)
                    {
                        //Buffer.BlockCopy(rawDepthImage, 0, sensorData.depthImage, 0, rawDepthImage.Length * sizeof(ushort));
                        KinectInterop.CopyBytes(rawDepthImage, sizeof(ushort), sensorData.depthImage, sizeof(ushort));
                    }

                    sensorData.lastDepthFrameTime = currentDepthTimestamp;
                    //Debug.Log("Depth frame. Timestamp: " + currentDepthTimestamp);
                }
            }

            // depth hist frame
            lock(depthImageDataLock)
            {
                if (equalHistBufferData != null && sensorData.lastDepthHistTime != lastDepthImageTimestamp)
                {
                    if (sensorData.depthHistBufferData != null)
                    {
                        KinectInterop.CopyBytes(equalHistBufferData, sizeof(int), sensorData.depthHistBufferData, sizeof(int));
                    }

                    sensorData.depthHistTotalPoints = histDataTotalPoints;
                    sensorData.lastDepthHistTime = lastDepthImageTimestamp;
                    //Debug.Log("Depth hist frame. Timestamp: " + lastDepthImageTimestamp);
                }
            }

            // infrared frame
            lock (infraredFrameLock)
            {
                if (rawInfraredImage != null && sensorData.lastInfraredFrameTime != currentInfraredTimestamp)
                {
                    if (sensorData.infraredImage != null)
                    {
                        //Buffer.BlockCopy(rawInfraredImage, 0, sensorData.infraredImage, 0, rawInfraredImage.Length * sizeof(ushort));
                        KinectInterop.CopyBytes(rawInfraredImage, sizeof(ushort), sensorData.infraredImage, sizeof(ushort));
                    }

                    sensorData.lastInfraredFrameTime = currentInfraredTimestamp;
                    //Debug.Log("IR frame. Timestamp: " + currentDepthTimestamp);
                }
            }

            return true;
        }


        // returns the point cloud texture resolution
        protected Vector2Int GetPointCloudTexResolution(KinectInterop.SensorData sensorData)
        {
            Vector2Int texRes = Vector2Int.zero;

            switch (pointCloudResolution)
            {
                case PointCloudResolution.DepthCameraResolution:
                    texRes = new Vector2Int(sensorData.depthImageWidth, sensorData.depthImageHeight);
                    break;

                case PointCloudResolution.ColorCameraResolution:
                    texRes = new Vector2Int(sensorData.colorImageWidth, sensorData.colorImageHeight);
                    break;
            }

            if(texRes == Vector2Int.zero)
            {
                throw new Exception("Unsupported point cloud resolution: " + pointCloudResolution + " or the respective image is not available.");
            }

            return texRes;
        }


        // creates the point-cloud vertex shader and its respective buffers, as needed
        protected virtual bool CreatePointCloudVertexShader(KinectInterop.SensorData sensorData)
        {
            if (sensorData.depthCamIntr == null || sensorData.depthCamIntr.distType == KinectInterop.DistortionType.None)
                return false;

            pointCloudVertexRes = GetPointCloudTexResolution(sensorData);

            if (pointCloudVertexRT == null)
            {
                pointCloudVertexRT = new RenderTexture(pointCloudVertexRes.x, pointCloudVertexRes.y, 0, RenderTextureFormat.ARGBHalf);
                pointCloudVertexRT.enableRandomWrite = true;
                pointCloudVertexRT.Create();
            }

            if (pointCloudVertexShader == null)
            {
                pointCloudVertexShader = Resources.Load("PointCloudVertexShaderAll") as ComputeShader;
                pointCloudVertexKernel = pointCloudVertexShader != null ? pointCloudVertexShader.FindKernel("BakeVertexTex") : -1;
            }

            if (pointCloudSpaceBuffer == null)
            {
                int spaceBufferLength = pointCloudVertexRes.x * pointCloudVertexRes.y * 3;
                pointCloudSpaceBuffer = new ComputeBuffer(spaceBufferLength, sizeof(float));

                // depth2space table
                int depthImageLength = pointCloudVertexRes.x * pointCloudVertexRes.y;
                Vector3[] depth2SpaceTable = new Vector3[depthImageLength];

                for (int dy = 0, di = 0; dy < pointCloudVertexRes.y; dy++)
                {
                    for (int dx = 0; dx < pointCloudVertexRes.x; dx++, di++)
                    {
                        Vector2 depthPos = new Vector2(dx, dy);
                        depth2SpaceTable[di] = pointCloudResolution == PointCloudResolution.ColorCameraResolution ?
                            MapColorPointToSpaceCoords(sensorData, depthPos, 1000) : MapDepthPointToSpaceCoords(sensorData, depthPos, 1000);
                    }
                }

                pointCloudSpaceBuffer.SetData(depth2SpaceTable);
                depth2SpaceTable = null;
            }

            if (pointCloudDepthBuffer == null)
            {
                int depthBufferLength = pointCloudVertexRes.x * pointCloudVertexRes.y / 2;
                pointCloudDepthBuffer = new ComputeBuffer(depthBufferLength, sizeof(uint));
            }

            if (pointCloudResolution == PointCloudResolution.ColorCameraResolution && color2DepthDataFrame == null)
            {
                color2DepthDataFrame = new ushort[sensorData.colorImageWidth * sensorData.colorImageHeight];
            }

            return true;
        }


        // disposes the point-cloud vertex shader and its respective buffers
        protected virtual void DisposePointCloudVertexShader(KinectInterop.SensorData sensorData)
        {
            if (pointCloudSpaceBuffer != null)
            {
                pointCloudSpaceBuffer.Dispose();
                pointCloudSpaceBuffer = null;
            }

            if (pointCloudDepthBuffer != null)
            {
                pointCloudDepthBuffer.Dispose();
                pointCloudDepthBuffer = null;
            }

            if (pointCloudVertexRT != null)
            {
                pointCloudVertexRT.Release();
                pointCloudVertexRT = null;
            }

            if (color2DepthDataFrame != null)
            {
                color2DepthDataFrame = null;
            }

            if (pointCloudVertexShader != null)
            {
                pointCloudVertexShader = null;
            }
        }


        // updates the point-cloud vertex shader with the actual data
        protected virtual bool UpdatePointCloudVertexShader(KinectInterop.SensorData sensorData)
        {
            if (pointCloudVertexShader != null && sensorData.depthImage != null && pointCloudVertexRT != null &&
                sensorData.lastDepth2SpaceFrameTime != sensorData.lastDepthFrameTime)
            {
                sensorData.lastDepth2SpaceFrameTime = sensorData.lastDepthFrameTime;

                if (pointCloudResolution == PointCloudResolution.ColorCameraResolution)
                {
                    lock(color2DepthFrameLock)
                    {
                        KinectInterop.SetComputeBufferData(pointCloudDepthBuffer, color2DepthDataFrame, color2DepthDataFrame.Length >> 1, sizeof(uint));
                    }
                }
                else
                {
                    KinectInterop.SetComputeBufferData(pointCloudDepthBuffer, sensorData.depthImage, sensorData.depthImage.Length >> 1, sizeof(uint));
                }

                KinectInterop.SetComputeShaderInt2(pointCloudVertexShader, "DepthRes", pointCloudVertexRes.x, pointCloudVertexRes.y);
                KinectInterop.SetComputeShaderFloat2(pointCloudVertexShader, "DepthScale", sensorData.sensorSpaceScale.x, sensorData.sensorSpaceScale.y);
                pointCloudVertexShader.SetInt("MinDepth", (int)(minDistance * 1000f));
                pointCloudVertexShader.SetInt("MaxDepth", (int)(maxDistance * 1000f));
                pointCloudVertexShader.SetBuffer(pointCloudVertexKernel, "SpaceTable", pointCloudSpaceBuffer);
                pointCloudVertexShader.SetBuffer(pointCloudVertexKernel, "DepthMap", pointCloudDepthBuffer);
                pointCloudVertexShader.SetTexture(pointCloudVertexKernel, "PointCloudVertexTex", pointCloudVertexRT);
                pointCloudVertexShader.Dispatch(pointCloudVertexKernel, pointCloudVertexRes.x / 8, pointCloudVertexRes.y / 8, 1);

                if (pointCloudVertexTexture != null)
                {
                    Graphics.Blit(pointCloudVertexRT, pointCloudVertexTexture);
                }

                return true;
            }

            return false;
        }


        // creates the point-cloud color shader and its respective buffers, as needed
        protected virtual bool CreatePointCloudColorShader(KinectInterop.SensorData sensorData)
        {
            pointCloudColorRes = GetPointCloudTexResolution(sensorData);

            if(pointCloudResolution == PointCloudResolution.DepthCameraResolution)
            {
                if (pointCloudAlignedColorTex == null)
                {
                    pointCloudAlignedColorTex = new Texture2D(sensorData.depthImageWidth, sensorData.depthImageHeight, sensorData.colorImageFormat, false);
                }

                if (depth2ColorDataFrame == null)
                {
                    depth2ColorDataFrame = new byte[sensorData.depthImageWidth * sensorData.depthImageHeight * sensorData.colorImageStride];
                }
            }

            return true;
        }


        // disposes the point-cloud color shader and its respective buffers
        protected virtual void DisposePointCloudColorShader(KinectInterop.SensorData sensorData)
        {
            if (pointCloudCoordBuffer != null)
            {
                pointCloudCoordBuffer.Dispose();
                pointCloudCoordBuffer = null;
            }

            if (pointCloudColorRT)
            {
                pointCloudColorRT.Release();
                pointCloudColorRT = null;
            }

            if (pointCloudAlignedColorTex != null)
            {
                Destroy(pointCloudAlignedColorTex);
                pointCloudAlignedColorTex = null;
            }

            if (depth2ColorDataFrame != null)
            {
                depth2ColorDataFrame = null;
            }

            if (depth2ColorCoordFrame != null)
            {
                depth2ColorCoordFrame = null;
            }

            if (pointCloudColorShader != null)
            {
                pointCloudColorShader = null;
            }
        }


        // updates the point-cloud color shader with the actual data
        protected virtual bool UpdatePointCloudColorShader(KinectInterop.SensorData sensorData)
        {
            Texture texColor = null;

            if (pointCloudResolution == PointCloudResolution.DepthCameraResolution)
            {
                if (pointCloudAlignedColorTex != null && depth2ColorDataFrame != null && sensorData.lastDepth2ColorFrameTime != lastDepth2ColorFrameTime)
                {
                    lock (depth2ColorFrameLock)
                    {
                        sensorData.lastDepth2ColorFrameTime = lastDepth2ColorFrameTime;

                        pointCloudAlignedColorTex.LoadRawTextureData(depth2ColorDataFrame);
                        pointCloudAlignedColorTex.Apply();
                    }

                    if (pointCloudColorRT != null)
                    {
                        Graphics.CopyTexture(pointCloudAlignedColorTex, pointCloudColorRT);
                    }

                    texColor = pointCloudAlignedColorTex;
                }
            }
            else
            {
                texColor = sensorData.colorImageTexture;
            }

            if(texColor != null)
            {
                Graphics.Blit(texColor, pointCloudColorTexture);
                return true;
            }

            return false;
        }


        // updates the coordinate mapping data, as needed
        public virtual bool UpdateFrameTransformData(KinectInterop.SensorData sensorData, KinectManager kinectManager)
        {
            // depth2space frame
            if (pointCloudVertexTexture != null)
            {
                if (pointCloudVertexShader != null || CreatePointCloudVertexShader(sensorData))
                {
                    UpdatePointCloudVertexShader(sensorData);
                }
            }
            else
            {
                if (pointCloudVertexShader != null)
                {
                    DisposePointCloudVertexShader(sensorData);
                }
            }

            // depth2color frame
            if (pointCloudColorTexture != null)
            {
                if (pointCloudColorShader != null || pointCloudAlignedColorTex != null || CreatePointCloudColorShader(sensorData))
                {
                    UpdatePointCloudColorShader(sensorData);
                }
            }
            else
            {
                if (pointCloudColorShader != null || pointCloudAlignedColorTex != null)
                {
                    DisposePointCloudColorShader(sensorData);
                }
            }

            return true;
        }


        public virtual bool UpdateSensorTextures(KinectInterop.SensorData sensorData, KinectManager kinectManager, ulong prevDepthFrameTime)
        {
            // check if the depth data has changed
            if (prevDepthFrameTime != sensorData.lastDepthFrameTime)
            {
                if (sensorData.depthImageTexture != null && sensorData.depthImageMaterial != null)
                {
                    // depth texture
                    if (sensorData.depthImageBuffer != null)
                    {
                        int depthBufferLength = sensorData.depthImageWidth * sensorData.depthImageHeight / 2;
                        KinectInterop.SetComputeBufferData(sensorData.depthImageBuffer, sensorData.depthImage, depthBufferLength, sizeof(uint));
                    }

                    if (sensorData.depthHistBuffer != null && sensorData.depthHistBufferData != null)
                    {
                        //sensorData.depthHistBuffer.SetData(equalHistBufferData);
                        KinectInterop.SetComputeBufferData(sensorData.depthHistBuffer, sensorData.depthHistBufferData, sensorData.depthHistBufferData.Length, sizeof(int));
                    }

                    sensorData.depthImageMaterial.SetInt("_TexResX", sensorData.depthImageWidth);
                    sensorData.depthImageMaterial.SetInt("_TexResY", sensorData.depthImageHeight);
                    sensorData.depthImageMaterial.SetInt("_MinDepth", (int)(minDistance * 1000f));
                    sensorData.depthImageMaterial.SetInt("_MaxDepth", (int)(maxDistance * 1000f));
                    sensorData.depthImageMaterial.SetInt("_TotalPoints", sensorData.depthHistTotalPoints);
                    sensorData.depthImageMaterial.SetBuffer("_DepthMap", sensorData.depthImageBuffer);
                    sensorData.depthImageMaterial.SetBuffer("_HistMap", sensorData.depthHistBuffer);

                    Graphics.Blit(null, sensorData.depthImageTexture, sensorData.depthImageMaterial);
                    //Debug.Log("Depth texture. Timestamp: " + sensorData.lastDepthFrameTime);
                }
            }

            return true;
        }


        // returns sensor-to-world matrix
        public virtual Matrix4x4 GetSensorToWorldMatrix()
        {
            Matrix4x4 mSensor = Matrix4x4.identity;
            mSensor.SetTRS(sensorPosePosition, sensorPoseRotation, Vector3.one);

            return mSensor;
        }


        // returns sensor transform. Please note transform updates depend on the getPoseFrames-KM setting.
        public virtual Transform GetSensorTransform()
        {
            return transform;
        }


        // unprojects plane point into the space
        protected virtual Vector3 UnprojectPoint(KinectInterop.CameraIntrinsics intr, Vector2 pixel, float depth)
        {
            return Vector3.zero;
        }


        // projects space point onto a plane
        protected virtual Vector2 ProjectPoint(KinectInterop.CameraIntrinsics intr, Vector3 point)
        {
            return Vector2.zero;
        }


        // transforms a point from one space to another
        protected virtual Vector3 TransformPoint(KinectInterop.CameraExtrinsics extr, Vector3 point)
        {
            return Vector3.zero;
        }


        public virtual Vector3 MapDepthPointToSpaceCoords(KinectInterop.SensorData sensorData, Vector2 depthPos, ushort depthVal)
        {
            if (sensorData.depthCamIntr != null)
            {
                return UnprojectPoint(sensorData.depthCamIntr, depthPos, (float)depthVal / 1000f);
            }

            return Vector3.zero;
        }


        public virtual Vector2 MapSpacePointToDepthCoords(KinectInterop.SensorData sensorData, Vector3 spacePos)
        {
            if (sensorData.depthCamIntr != null)
            {
                return ProjectPoint(sensorData.depthCamIntr, spacePos);
            }

            return Vector2.zero;
        }


        public virtual Vector3 MapColorPointToSpaceCoords(KinectInterop.SensorData sensorData, Vector2 colorPos, ushort depthVal)
        {
            if (sensorData.colorCamIntr != null)
            {
                return UnprojectPoint(sensorData.colorCamIntr, colorPos, (float)depthVal / 1000f);
            }

            return Vector3.zero;
        }


        public virtual Vector2 MapSpacePointToColorCoords(KinectInterop.SensorData sensorData, Vector3 spacePos)
        {
            if (sensorData.colorCamIntr != null)
            {
                return ProjectPoint(sensorData.colorCamIntr, spacePos);
            }

            return Vector2.zero;
        }


        public virtual Vector2 MapDepthPointToColorCoords(KinectInterop.SensorData sensorData, Vector2 depthPos, ushort depthVal)
        {
            if (sensorData.depthCamIntr != null && sensorData.colorCamIntr != null && sensorData.depth2ColorExtr != null)
            {
                Vector3 depthSpacePos = UnprojectPoint(sensorData.depthCamIntr, depthPos, (float)depthVal / 1000f);
                Vector3 colorSpacePos = TransformPoint(sensorData.depth2ColorExtr, depthSpacePos);
                Vector2 colorPos = ProjectPoint(sensorData.colorCamIntr, colorSpacePos);

                return colorPos;
            }

            return Vector2.zero;
        }


        // estimates horizontal and vertical FOV
        protected void EstimateFOV(KinectInterop.CameraIntrinsics intr)
        {
            intr.hFOV = (Mathf.Atan2(intr.ppx + 0.5f, intr.fx) + Mathf.Atan2(intr.width - (intr.ppx + 0.5f), intr.fx)) * 57.2957795f;
            intr.vFOV = (Mathf.Atan2(intr.ppy + 0.5f, intr.fy) + Mathf.Atan2(intr.height - (intr.ppy + 0.5f), intr.fy)) * 57.2957795f;
        }

    }
}
