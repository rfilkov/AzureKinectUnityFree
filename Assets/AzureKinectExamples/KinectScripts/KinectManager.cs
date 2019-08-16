using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace com.rfilkov.kinect
{
    /// <summary>
    /// KinectManager is the the main and most basic depth-sensor related component. It controls the sensors and manages the data streams.
    /// </summary>
    public class KinectManager : MonoBehaviour
    {

        [Header("Sensor Data")]

        [Tooltip("Whether to get depth frames from the sensor(s).")]
        public DepthTextureType getDepthFrames = DepthTextureType.RawDepthData;
        public enum DepthTextureType : int { None = 0, RawDepthData = 1, DepthTexture = 2 }

        [Tooltip("Whether to get color frames from the sensor(s).")]
        public ColorTextureType getColorFrames = ColorTextureType.None;
        public enum ColorTextureType : int { None = 0, ColorTexture = 2 }

        [Tooltip("Whether to get infrared frames from the sensor(s).")]
        public InfraredTextureType getInfraredFrames = InfraredTextureType.None;
        public enum InfraredTextureType : int { None = 0, RawInfraredData = 1 }

        [Tooltip("Whether to get pose frames from the sensor(s).")]
        public PoseUsageType getPoseFrames = PoseUsageType.None;
        public enum PoseUsageType : int { None = 0, RawPoseData = 1 }

        [Tooltip("Whether to poll the sensor frames in separate threads or in the Update-method.")]
        private bool pollFramesInThread = true;

        [Tooltip("Whether to synchronize depth and color frames.")]
        public bool syncDepthAndColor = false;

        [Header("On-Screen Info")]

        [Tooltip("List of images to display on the screen.")]
        public List<DisplayImageType> displayImages = new List<DisplayImageType>();
        public enum DisplayImageType : int
        {
            Sensor0ColorImage = 0x01, Sensor0DepthImage = 0x02,
            Sensor1ColorImage = 0x11, Sensor1DepthImage = 0x12,
            Sensor2ColorImage = 0x21, Sensor2DepthImage = 0x22,
            UserBodyImage = 0x101
        }

        [Tooltip("Single image width, as percent of the screen width. The height is estimated according to the image's aspect ratio.")]
        [Range(0.1f, 0.5f)]
        public float displayImageWidthPercent = 0.2f;

        [Tooltip("UI-Text to display status messages.")]
        public UnityEngine.UI.Text statusInfoText;



        // Bool to keep track of whether Kinect has been initialized
        protected bool kinectInitialized = false;

        // The singleton instance of KinectManager
        protected static KinectManager instance = null;

        // available sensor interfaces
        protected List<DepthSensorInterface> sensorInterfaces = new List<DepthSensorInterface>();
        // the respective SensorData structures
        protected List<KinectInterop.SensorData> sensorDatas = new List<KinectInterop.SensorData>();


        /// <summary>
        /// Gets the single KinectManager instance.
        /// </summary>
        /// <value>The KinectManager instance.</value>
        public static KinectManager Instance
        {
            get
            {
                return instance;
            }
        }

        /// <summary>
        /// Determines if the KinectManager-component is initialized and ready to use.
        /// </summary>
        /// <returns><c>true</c> if KinectManager is initialized; otherwise, <c>false</c>.</returns>
        public bool IsInitialized()
        {
            return kinectInitialized;
        }

        /// <summary>
        /// Returns the number of utilized depth sensors.
        /// </summary>
        /// <returns>The number of depth sensors.</returns>
        public int GetSensorCount()
        {
            return sensorDatas.Count;
        }

        ///// <summary>
        ///// Gets the sensor-data structure of the 1st sensor (this structure should not be modified, because it is used internally).
        ///// </summary>
        ///// <returns>The sensor data.</returns>
        //internal KinectInterop.SensorData GetSensorData()
        //{
        //    return GetSensorData(0);
        //}

        /// <summary>
        /// Gets the sensor-data structure of the given sensor (this structure should not be modified, because it is used internally).
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The sensor data.</returns>
        internal KinectInterop.SensorData GetSensorData(int sensorIndex)
        {
            if(sensorIndex >= 0  && sensorIndex < sensorDatas.Count)
            {
                return sensorDatas[sensorIndex];
            }

            return null;
        }

        /// <summary>
        /// Gets the minimum distance tracked by the sensor, in meters.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>Minimum distance tracked by the sensor, in meters.</returns>
        public float GetSensorMinDistance(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if(sensorData != null && sensorData.sensorInterface != null)
            {
                return ((DepthSensorBase)sensorData.sensorInterface).minDistance;
            }

            return 0f;
        }

        /// <summary>
        /// Gets the maximum distance tracked by the sensor, in meters.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>Maximum distance tracked by the sensor, in meters.</returns>
        public float GetSensorMaxDistance(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null && sensorData.sensorInterface != null)
            {
                return ((DepthSensorBase)sensorData.sensorInterface).maxDistance;
            }

            return 0f;
        }

        /// <summary>
        /// Gets the last color frame time, as returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The color frame time.</returns>
        public ulong GetColorFrameTime(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.lastColorFrameTime : 0;
        }

        /// <summary>
        /// Gets the width of the color image, returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The color image width.</returns>
        public int GetColorImageWidth(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.colorImageWidth : 0;
        }

        /// <summary>
        /// Gets the height of the color image, returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The color image height.</returns>
        public int GetColorImageHeight(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.colorImageHeight : 0;
        }

        /// <summary>
        /// Gets the color image scale.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The color image scale.</returns>
        public Vector3 GetColorImageScale(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.colorImageScale : Vector3.one;
        }

        /// <summary>
        /// Gets the color image texture.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The color image texture.</returns>
        public Texture GetColorImageTex(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.colorImageTexture : null;
        }

        /// <summary>
        /// Gets the last depth frame time, as returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The depth frame time.</returns>
        public ulong GetDepthFrameTime(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.lastDepthFrameTime : 0;
        }

        /// <summary>
        /// Gets the last IR frame time, as returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The IR frame time.</returns>
        public ulong GetInfraredFrameTime(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.lastInfraredFrameTime : 0;
        }

        /// <summary>
        /// Gets the width of the depth image, returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The depth image width.</returns>
        public int GetDepthImageWidth(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.depthImageWidth : 0;
        }

        /// <summary>
        /// Gets the height of the depth image, returned by the sensor.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The depth image height.</returns>
        public int GetDepthImageHeight(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.depthImageHeight : 0;
        }

        /// <summary>
        /// Gets the raw depth data, if ComputeUserMap is true.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The raw depth map.</returns>
        public ushort[] GetRawDepthMap(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.depthImage : null;
        }

        /// <summary>
        /// Gets the raw infrared data, if ComputeInfraredMap is true.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The raw infrared map.</returns>
        public ushort[] GetRawInfraredMap(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.infraredImage : null;
        }

        /// <summary>
        /// Gets the depth image scale.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The depth image scale.</returns>
        public Vector3 GetDepthImageScale(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.depthImageScale : Vector3.one;
        }

        /// <summary>
        /// Gets the infrared image scale.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The infrared image scale.</returns>
        public Vector3 GetInfraredImageScale(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.infraredImageScale : Vector3.one;
        }

        /// <summary>
        /// Gets the sensor space scale.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The sensor space scale.</returns>
        public Vector3 GetSensorSpaceScale(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.sensorSpaceScale : Vector3.one;
        }

        /// <summary>
        /// Gets the depth image texture.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>The depth texture.</returns>
        public Texture GetDepthImageTex(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return sensorData != null ? sensorData.depthImageTexture : null;
        }

        /// <summary>
        /// Gets the depth value for the specified pixel, if ComputeUserMap is true.
        /// </summary>
        /// <returns>The depth value.</returns>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <param name="x">The X coordinate of the pixel.</param>
        /// <param name="y">The Y coordinate of the pixel.</param>
        public ushort GetDepthForPixel(int sensorIndex, int x, int y)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null && sensorData.depthImage != null)
            {
                int index = y * sensorData.depthImageWidth + x;

                if (index >= 0 && index < sensorData.depthImage.Length)
                {
                    return sensorData.depthImage[index];
                }
            }

            return 0;
        }

        /// <summary>
        /// Gets the depth value for the specified pixel, if ComputeUserMap is true.
        /// </summary>
        /// <returns>The depth value.</returns>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <param name="index">Depth index.</param>
        public ushort GetDepthForIndex(int sensorIndex, int index)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null && sensorData.depthImage != null)
            {
                if (index >= 0 && index < sensorData.depthImage.Length)
                {
                    return sensorData.depthImage[index];
                }
            }

            return 0;
        }


        /// <summary>
        /// Returns the respective sensor-to-world matrix.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>Sensor-to-world matrix.</returns>
        public Matrix4x4 GetSensorToWorldMatrix(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return KinectInterop.GetSensorToWorldMatrix(sensorData);
        }


        /// <summary>
        /// Returns the sensor transform reference. Please note transform updates depend on the getPoseFrames-KM setting.
        /// </summary>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <returns>Sensor transorm or null, if sensorIndex is invalid.</returns>
        public Transform GetSensorTransform(int sensorIndex)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            return KinectInterop.GetSensorTransform(sensorData);
        }


        /// <summary>
        /// Returns the space coordinates of a depth-map point, or Vector3.zero if the sensor is not initialized
        /// </summary>
        /// <returns>The space coordinates.</returns>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <param name="posPoint">Depth point coordinates</param>
        /// <param name="depthValue">Depth value</param>
        /// <param name="bWorldCoords">If set to <c>true</c>, applies the sensor height and angle to the space coordinates.</param>
        public Vector3 MapDepthPointToSpaceCoords(int sensorIndex, Vector2 posPoint, ushort depthValue, bool bWorldCoords)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null)
            {
                Vector3 posSpace = KinectInterop.MapDepthPointToSpaceCoords(sensorData, posPoint, depthValue);

                if (bWorldCoords)
                {
                    Vector3 spaceScale = sensorData.sensorSpaceScale;
                    posSpace = new Vector3(posSpace.x * spaceScale.x, posSpace.y * spaceScale.y, posSpace.z * spaceScale.z);

                    Matrix4x4 sensorToWorld = KinectInterop.GetSensorToWorldMatrix(sensorData);
                    posSpace = sensorToWorld.MultiplyPoint3x4(posSpace);
                }

                return posSpace;
            }

            return Vector3.zero;
        }


        /// <summary>
        /// Returns the depth-map coordinates of a space point, or Vector2.zero if Kinect is not initialized
        /// </summary>
        /// <returns>The depth-map coordinates.</returns>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <param name="posPoint">Space point coordinates</param>
        public Vector2 MapSpacePointToDepthCoords(int sensorIndex, Vector3 posPoint)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null)
            {
                return KinectInterop.MapSpacePointToDepthCoords(sensorData, posPoint);
            }

            return Vector2.zero;
        }


        /// <summary>
        /// Returns the color-map coordinates of a depth point.
        /// </summary>
        /// <returns>The color-map coordinates.</returns>
        /// <param name="sensorIndex">The sensor index.</param>
        /// <param name="posPoint">Depth point coordinates</param>
        /// <param name="depthValue">Depth value</param>
        public Vector2 MapDepthPointToColorCoords(int sensorIndex, Vector2 posPoint, ushort depthValue)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData != null)
            {
                return KinectInterop.MapDepthPointToColorCoords(sensorData, posPoint, depthValue);
            }

            return Vector2.zero;
        }


        /// <summary>
        /// Gets the foreground rectangle of the depth image.
        /// </summary>
        /// <param name="sensorIndex">Sensor index.</param>
        /// <param name="foregroundCamera">The foreground camera, or null if there is no foreground camera.</param>
        /// <returns>The foreground rectangle.</returns>
        public Rect GetForegroundRectDepth(int sensorIndex, Camera foregroundCamera)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData == null)
                return new Rect();

            Rect cameraRect = foregroundCamera ? foregroundCamera.pixelRect : new Rect(0, 0, Screen.width, Screen.height);
            float rectHeight = cameraRect.height;
            float rectWidth = cameraRect.width;

            if (rectWidth > rectHeight)
                rectWidth = rectHeight * sensorData.depthImageWidth / sensorData.depthImageHeight;
            else
                rectHeight = rectWidth * sensorData.depthImageHeight / sensorData.depthImageWidth;

            float foregroundOfsX = (cameraRect.width - rectWidth) / 2;
            float foregroundOfsY = (cameraRect.height - rectHeight) / 2;

            Rect foregroundImgRect = new Rect(foregroundOfsX, foregroundOfsY, rectWidth, rectHeight);

            return foregroundImgRect;
        }

        /// <summary>
        /// Gets the foreground rectangle of the color image..
        /// </summary>
        /// <param name="sensorIndex">Sensor index.</param>
        /// <param name="foregroundCamera">The foreground camera, or null if there is no foreground camera.</param>
        /// <returns>The foreground rectangle.</returns>
        public Rect GetForegroundRectColor(int sensorIndex, Camera foregroundCamera)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);
            if (sensorData == null)
                return new Rect();

            Rect cameraRect = foregroundCamera ? foregroundCamera.pixelRect : new Rect(0, 0, Screen.width, Screen.height);
            float rectHeight = cameraRect.height;
            float rectWidth = cameraRect.width;

            if (rectWidth > rectHeight)
                rectWidth = rectHeight * sensorData.colorImageWidth / sensorData.colorImageHeight;
            else
                rectHeight = rectWidth * sensorData.colorImageHeight / sensorData.colorImageWidth;

            float foregroundOfsX = (cameraRect.width - rectWidth) / 2;
            float foregroundOfsY = (cameraRect.height - rectHeight) / 2;

            Rect foregroundImgRect = new Rect(foregroundOfsX, foregroundOfsY, rectWidth, rectHeight);

            return foregroundImgRect;
        }

        /// <summary>
        /// Gets the 3d overlay position of a point over the depth-image.
        /// </summary>
        /// <returns>The 3d position for depth overlay.</returns>
        /// <param name="dx">Depth image X</param>
        /// <param name="dy">Depth image X</param>
        /// <param name="depth">Distance in mm. If it is 0, the function will try to read the current depth value.</param>
        /// <param name="camera">Camera used to visualize the 3d overlay position</param>
        /// <param name="imageRect">Depth image rectangle on the screen</param>
        public Vector3 GetPosDepthOverlay(int dx, int dy, ushort depth, int sensorIndex, Camera camera, Rect imageRect)
        {
            KinectInterop.SensorData sensorData = GetSensorData(sensorIndex);

            float xScaled = (float)dx * imageRect.width / sensorData.depthImageWidth;
            float yScaled = (float)dy * imageRect.height / sensorData.depthImageHeight;

            float xScreen = imageRect.x + (sensorData.depthImageScale.x > 0f ? xScaled : imageRect.width - xScaled);
            float yScreen = imageRect.y + (sensorData.depthImageScale.y > 0f ? yScaled : imageRect.height - yScaled);

            if(depth == 0)
            {
                depth = sensorData.depthImage[dx + dy * sensorData.depthImageWidth];
            }

            if (depth != 0)
            {
                float zDistance = (float)depth / 1000f;
                Vector3 vPosJoint = camera.ScreenToWorldPoint(new Vector3(xScreen, yScreen, zDistance));

                return vPosJoint;
            }

            return Vector3.zero;
        }


        // internal methods

        void Awake()
        {
            // initializes the singleton instance of KinectManager
            if (instance == null)
            {
                instance = this;
                DontDestroyOnLoad(this);
            }
            else if (instance != this)
            {
                DestroyImmediate(gameObject);
                return;
            }

            // set graphics shader level
            KinectInterop.SetGraphicsShaderLevel(SystemInfo.graphicsShaderLevel);

            // locate and start the available depth-sensors
            StartDepthSensors();
        }


        // gets the frame-source flags
        private KinectInterop.FrameSource GetFrameSourceFlags()
        {
            KinectInterop.FrameSource dwFlags = KinectInterop.FrameSource.TypeNone;

            if (getDepthFrames != DepthTextureType.None)
                dwFlags |= KinectInterop.FrameSource.TypeDepth;
            if (getColorFrames != ColorTextureType.None)
                dwFlags |= KinectInterop.FrameSource.TypeColor;
            if (getInfraredFrames != InfraredTextureType.None)
                dwFlags |= KinectInterop.FrameSource.TypeInfrared;
            if (getPoseFrames != PoseUsageType.None)
                dwFlags |= KinectInterop.FrameSource.TypePose;

            return dwFlags;
        }


        // locates and starts the available depth-sensors and their interfaces
        private void StartDepthSensors()
        {
            try
            {
                // try to initialize the available sensors
                KinectInterop.FrameSource dwFlags = GetFrameSourceFlags();

                // locate the available depth-sensor interfaces in the scene
                List<DepthSensorBase> sensorInts = new List<DepthSensorBase>();
                sensorInts.AddRange(gameObject.GetComponents<DepthSensorBase>());  // FindObjectsOfType<MonoBehaviour>();
                sensorInts.AddRange(gameObject.GetComponentsInChildren<DepthSensorBase>());

                if (sensorInts.Count == 0)
                {
                    // by-default add K4A interface
                    transform.position = new Vector3(0f, 1f, 0f);
                    transform.rotation = Quaternion.identity;

                    DepthSensorBase sensorInt = gameObject.AddComponent<Kinect4AzureInterface>();
                    sensorInts.Add(sensorInt);
                }

                for (int i = 0; i < sensorInts.Count; i++)
                {
                    if (sensorInts[i] is DepthSensorBase)
                    {
                        DepthSensorBase sensorInt = (DepthSensorBase)sensorInts[i];
                        if(!sensorInt.enabled || sensorInt.deviceStreamingMode == KinectInterop.DeviceStreamingMode.Disabled || sensorInt.deviceIndex < 0)
                        {
                            Debug.Log(string.Format("S{0}: {1} disabled.", i, sensorInt.GetType().Name));
                            continue;
                        }

                        try
                        {
                            Debug.Log(string.Format("Opening S{0}: {1}, device-index: {2}", i, sensorInt.GetType().Name, sensorInt.deviceIndex));
                            KinectInterop.SensorData sensorData = sensorInt.OpenSensor(dwFlags, syncDepthAndColor, false);

                            if(sensorData != null)
                            {
                                //Debug.Log("Succeeded opening " + sensorInt.GetType().Name);

                                sensorData.sensorInterface = sensorInt;
                                KinectInterop.InitSensorData(sensorData, this);

                                sensorInterfaces.Add(sensorInt);
                                sensorDatas.Add(sensorData);

                                if(pollFramesInThread)
                                {
                                    sensorData.threadStopEvent = new AutoResetEvent(false);
                                    sensorData.pollFramesThread = new Thread(() => PollFramesThread(sensorData));
                                    sensorData.pollFramesThread.IsBackground = true;
                                    sensorData.pollFramesThread.Start();
                                }
                            }
                        }
                        catch(Exception ex)
                        {
                            Debug.LogException(ex);
                            Debug.LogError("Failed opening " + sensorInt.GetType().Name + ", device-index: " + sensorInt.deviceIndex);
                        }
                    }
                }

                Debug.Log(string.Format("{0} sensor(s) opened.", sensorDatas.Count));

                // set initialization status
                if (sensorInterfaces.Count > 0)
                {
                    kinectInitialized = true;
                }
                else
                {
                    kinectInitialized = false;

                    string sErrorMessage = "No suitable depth-sensor found. Please check the connected devices and installed SDKs.";
                    Debug.LogError(sErrorMessage);

                    if (statusInfoText != null)
                    {
                        statusInfoText.text = sErrorMessage;
                    }
                }
            }
            //catch (DllNotFoundException ex)
            //{
            //    string message = ex.Message + " cannot be loaded. Please check the respective SDK installation.";

            //    Debug.LogError(message);
            //    Debug.LogException(ex);

            //    if (calibrationText != null)
            //    {
            //        calibrationText.text = message;
            //    }

            //    return;
            //}
            catch (Exception ex)
            {
                string message = ex.Message;

                Debug.LogError(message);
                Debug.LogException(ex);

                if (statusInfoText != null)
                {
                    statusInfoText.text = message;
                }

                return;
            }
        }


        // polls for frames and updates the depth-sensor data in a thread
        private void PollFramesThread(KinectInterop.SensorData sensorData)
        {
            if (sensorData == null)
                return;

            while (!sensorData.threadStopEvent.WaitOne(0))
            {
                if (kinectInitialized)
                {
                    KinectInterop.PollSensorFrames(sensorData);
                }
            }
        }


        void OnApplicationQuit()
        {
            OnDestroy();
        }


        void OnDestroy()
        {
            if (instance == null || instance != this)
                return;

            //Debug.Log("KM was destroyed");

            // shut down the polling threads and stop the sensors
            if (kinectInitialized)
            {
                // close the opened sensors and release respective data
                for(int i = sensorDatas.Count - 1; i >= 0; i--)
                {
                    KinectInterop.SensorData sensorData = sensorDatas[i];
                    DepthSensorInterface sensorInt = sensorData.sensorInterface;
                    Debug.Log(string.Format("Closing S{0}: {1}", i, sensorInt.GetType().Name));

                    if (sensorData.pollFramesThread != null)
                    {
                        // stop the frame-polling thread
                        sensorData.threadStopEvent.Set();
                        sensorData.pollFramesThread.Join();

                        sensorData.pollFramesThread = null;
                        sensorData.threadStopEvent.Dispose();
                        sensorData.threadStopEvent = null;
                    }

                    // close the sensor
                    KinectInterop.CloseSensor(sensorData);

                    sensorDatas.RemoveAt(i);
                    sensorInterfaces.RemoveAt(i);
                }

                kinectInitialized = false;
            }

            instance = null;
        }


        void Update()
        {
            if (!kinectInitialized)
                return;

            if (!pollFramesInThread)
            {
                for (int i = 0; i < sensorDatas.Count; i++)
                {
                    KinectInterop.SensorData sensorData = sensorDatas[i];
                    KinectInterop.PollSensorFrames(sensorData);
                }
            }

            // update the sensor data, as needed
            for (int i = 0; i < sensorDatas.Count; i++)
            {
                KinectInterop.SensorData sensorData = sensorDatas[i];
                KinectInterop.UpdateSensorData(sensorData, this);
            }

            // update the sensor textures, if needed
            for (int i = 0; i < sensorDatas.Count; i++)
            {
                KinectInterop.UpdateSensorTextures(sensorDatas[i], this);
            }
        }


        void OnGUI()
        {
            if (!kinectInitialized)
                return;

            // display the selected images on screen
            for (int i = 0; i < displayImages.Count; i++)
            {
                Vector2 imageScale = Vector3.one;
                Texture imageTex = null;

                DisplayImageType imageType = displayImages[i];
                switch (imageType)
                {
                    case DisplayImageType.Sensor0ColorImage:
                    case DisplayImageType.Sensor1ColorImage:
                    case DisplayImageType.Sensor2ColorImage:
                        int si = imageType == DisplayImageType.Sensor0ColorImage ? 0 : imageType == DisplayImageType.Sensor1ColorImage ? 1 : imageType == DisplayImageType.Sensor2ColorImage ? 2 : -1;
                        if (si >= 0 && si < sensorDatas.Count)
                        {
                            KinectInterop.SensorData sensorData = sensorDatas[si];
                            imageScale = sensorData.colorImageScale;
                            imageTex = sensorData.colorImageTexture;
                        }
                        break;

                    case DisplayImageType.Sensor0DepthImage:
                    case DisplayImageType.Sensor1DepthImage:
                    case DisplayImageType.Sensor2DepthImage:
                        si = imageType == DisplayImageType.Sensor0DepthImage ? 0 : imageType == DisplayImageType.Sensor1DepthImage ? 1 : imageType == DisplayImageType.Sensor2DepthImage ? 2 : -1;
                        if (si >= 0 && si < sensorDatas.Count)
                        {
                            KinectInterop.SensorData sensorData = sensorDatas[si];
                            imageScale = sensorData.depthImageScale;
                            imageTex = sensorData.depthImageTexture;
                        }
                        break;

                    case DisplayImageType.UserBodyImage:
                        si = 0;  // sensor 0
                        if (si >= 0 && si < sensorDatas.Count)
                        {
                            KinectInterop.SensorData sensorData = sensorDatas[si];
                            imageScale = sensorData.depthImageScale;
                            imageTex = sensorData.bodyImageTexture;
                        }
                        break;
                }

                // display the image on screen
                KinectInterop.DisplayGuiTexture(i, displayImageWidthPercent, imageScale, imageTex);
            }
        }

    }
}
