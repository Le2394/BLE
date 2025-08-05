using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FlexFinger : MonoBehaviour
{
    public Transform index1;
    public Transform palm;
    public Quaternion palmQuat;
    public ESP32BleReceiver bleReceiver;

    void Update()
    {
        if (bleReceiver != null)
        {
            string fingerMessage = bleReceiver.GetMessage();
            float[] quaternion = bleReceiver.GetQuaternion();

            palmQuat = new Quaternion(quaternion[3], quaternion[2], -quaternion[1], quaternion[0]);
            palm.transform.localRotation = Quaternion.Slerp(palm.transform.localRotation, palmQuat, Time.deltaTime * 10f);
            Debug.Log("Palm Quaternion: " + palmQuat);
            if (fingerMessage == null || fingerMessage.Length == 0)
            {
                return;
            }
            else
            {
                float value = float.Parse(fingerMessage);
                index1.transform.localRotation = Quaternion.Euler(0f, 0f, MapSensorToAngle(value));

            }
        }
    }
    public static float MapSensorToAngle(float sensor)
    {
        float a = -0.00008282f;
        float b = 0.2096f;
        float c = -146.7669f;

        return a * sensor * sensor + b * sensor + c;
    }
}
