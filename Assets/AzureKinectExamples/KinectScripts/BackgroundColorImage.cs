using UnityEngine;
using System.Collections;
using com.rfilkov.kinect;


namespace com.rfilkov.components
{
    /// <summary>
    /// Background color image is component that displays the color camera feed on RawImage texture, usually the scene background.
    /// </summary>
    public class BackgroundColorImage : MonoBehaviour
    {
        [Tooltip("Depth sensor index - 0 is the 1st one, 1 - the 2nd one, etc.")]
        public int sensorIndex = 0;

        [Tooltip("RawImage used to display the color camera feed.")]
        public UnityEngine.UI.RawImage backgroundImage;

        [Tooltip("Camera used to display the background image. Set it, if you'd like to allow background image to resize, to match the color image's aspect ratio.")]
        public Camera backgroundCamera;


        void Start()
        {
            if (backgroundImage == null)
            {
                backgroundImage = GetComponent<UnityEngine.UI.RawImage>();
            }
        }


        void Update()
        {
            KinectManager kinectManager = KinectManager.Instance;

            if (kinectManager && kinectManager.IsInitialized())
            {
                if (backgroundImage && (backgroundImage.texture == null))
                {
                    backgroundImage.texture = kinectManager.GetColorImageTex(sensorIndex);
                    backgroundImage.rectTransform.localScale = kinectManager.GetColorImageScale(sensorIndex);
                    backgroundImage.color = Color.white;

                    //Debug.Log("aPos: " + backgroundImage.rectTransform.anchoredPosition + ", aMin: " + backgroundImage.rectTransform.anchorMin +
                    //    ", aMax:" + backgroundImage.rectTransform.anchorMax + ", pivot: " + backgroundImage.rectTransform.pivot + 
                    //    ", size: " + backgroundImage.rectTransform.sizeDelta);

                    if (backgroundCamera != null)
                    {
                        // adjust image's size and position to match the stream aspect ratio
                        int depthImageWidth = kinectManager.GetColorImageWidth(sensorIndex);
                        int depthImageHeight = kinectManager.GetColorImageHeight(sensorIndex);

                        float cameraWidth = backgroundCamera.pixelRect.width;
                        float cameraHeight = backgroundCamera.pixelRect.height;

                        RectTransform rectImage = backgroundImage.rectTransform;
                        float rectWidth = (rectImage.anchorMin.x != rectImage.anchorMax.x) ? cameraWidth * (rectImage.anchorMax.x - rectImage.anchorMin.x) : rectImage.sizeDelta.x;
                        float rectHeight = (rectImage.anchorMin.y != rectImage.anchorMax.y) ? cameraHeight * (rectImage.anchorMax.y - rectImage.anchorMin.y) : rectImage.sizeDelta.y;

                        if (rectWidth > rectHeight)
                            rectWidth = rectHeight * depthImageWidth / depthImageHeight;
                        else
                            rectHeight = rectWidth * depthImageHeight / depthImageWidth;

                        Vector2 pivotOffset = (rectImage.pivot - new Vector2(0.5f, 0.5f)) * 2f;
                        Vector2 imageScale = (Vector2)kinectManager.GetColorImageScale(sensorIndex);
                        Vector2 anchorPos = rectImage.anchoredPosition + pivotOffset * imageScale * new Vector2(rectWidth, rectHeight);

                        if (rectImage.anchorMin.x != rectImage.anchorMax.x)
                        {
                            rectWidth = -(cameraWidth - rectWidth);
                        }

                        if (rectImage.anchorMin.y != rectImage.anchorMax.y)
                        {
                            rectHeight = -(cameraHeight - rectHeight);
                        }

                        rectImage.sizeDelta = new Vector2(rectWidth, rectHeight);
                        rectImage.anchoredPosition = anchorPos;
                    }
                }
            }
        }

    }
}

