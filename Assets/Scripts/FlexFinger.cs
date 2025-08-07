using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FlexFinger : MonoBehaviour
{
    public Transform index1;
    public Transform elbow;
    public Transform palm;
    public Quaternion palmQuat;
    public Quaternion elbowQuat;
    public ESP32BleReceiver bleReceiver;

    void Update()
    {
        if (bleReceiver != null)
        {
            string fingerMessage = bleReceiver.GetMessage();
            float[] palmQuaternion = bleReceiver.GetPalmQuaternion();
            float[] elbowQuaternion = bleReceiver.GetElbowQuaternion();

            elbowQuat = new Quaternion(elbowQuaternion[3], elbowQuaternion[2], -elbowQuaternion[1], elbowQuaternion[0]);
            palmQuat = new Quaternion(palmQuaternion[3], palmQuaternion[2], -palmQuaternion[1], palmQuaternion[0]);

            elbow.transform.localRotation = Quaternion.Slerp(elbow.transform.localRotation, elbowQuat, Time.deltaTime * 10f);
            palm.transform.localRotation = Quaternion.Slerp(palm.transform.localRotation, palmQuat, Time.deltaTime * 10f);
            //Debug.Log("Palm Quaternion: " + palmQuat);

            if (fingerMessage == null || fingerMessage.Length == 0)
            {
                return;
            }
            else
            {
                if (fingerMessage.StartsWith("ADC:") && float.TryParse(fingerMessage.Substring(4), out float value))
                {
                    index1.transform.localRotation = Quaternion.Euler(0f, 0f, MapSensorToAngle(value));
                    //Debug.Log("Flex: " + value);
                }
                else
                {
                    Debug.LogWarning("Invalid ADC format: " + fingerMessage);
                }
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
