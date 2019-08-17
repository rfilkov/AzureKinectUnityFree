using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using com.rfilkov.kinect;


namespace com.rfilkov.components
{
    /// <summary>
    /// This component makes the game object follow the transform of the given sensor.
    /// </summary>
    public class FollowSensorTransform : MonoBehaviour
    {
        [Tooltip("Depth sensor index - 0 is the 1st one, 1 - the 2nd one, etc.")]
        public int sensorIndex = 0;

        [Tooltip("Smooth factor used for the game object movement and rotation.")]
        public float smoothFactor = 0f;


        // reference to the KinectManager
        private KinectManager kinectManager = null;


        void Start()
        {
            // get reference to KinectManager
            kinectManager = KinectManager.Instance;
        }

        void Update()
        {
            if(kinectManager && kinectManager.IsInitialized())
            {
                Transform sensorTrans = kinectManager.GetSensorTransform(sensorIndex);

                if(sensorTrans)
                {
                    if(smoothFactor != 0f)
                    {
                        transform.position = Vector3.Lerp(transform.position, sensorTrans.position, smoothFactor * Time.deltaTime);
                        transform.rotation = Quaternion.Slerp(transform.rotation, sensorTrans.rotation, smoothFactor * Time.deltaTime);
                    }
                    else
                    {
                        transform.position = sensorTrans.position;
                        transform.rotation = sensorTrans.rotation;
                    }
                }
            }
        }
    }
}
