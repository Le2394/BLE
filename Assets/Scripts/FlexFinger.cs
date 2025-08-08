using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FlexFinger : MonoBehaviour
{
    public Transform index1;
    public Transform shoulder;
    public Transform elbow;
    public Transform palm;

    public Quaternion palmQuat;
    public Quaternion elbowQuat;
    public Quaternion shoulderQuat;

    public ESP32BleReceiver bleReceiver;

    void Update()
    {
        if (bleReceiver != null)
        {
            string fingerMessage = bleReceiver.GetMessage();
            float[] palmQuaternion = bleReceiver.GetPalmQuaternion();
            float[] elbowQuaternion = bleReceiver.GetElbowQuaternion();
            float[] shoulderQuaternion = bleReceiver.GetShoulderQuaternion();

            shoulderQuat = new Quaternion(shoulderQuaternion[1], shoulderQuaternion[3], shoulderQuaternion[2], -shoulderQuaternion[0]);
            elbowQuat = new Quaternion(-elbowQuaternion[1], -elbowQuaternion[3], -elbowQuaternion[2], elbowQuaternion[0]);
            palmQuat = new Quaternion(palmQuaternion[1], palmQuaternion[3], palmQuaternion[2], -palmQuaternion[0]);

            shoulder.transform.rotation = Quaternion.Slerp(shoulder.transform.rotation, shoulderQuat, Time.deltaTime * 10f);
            elbow.transform.rotation = Quaternion.Slerp(elbow.transform.rotation, elbowQuat, Time.deltaTime * 10f);
            palm.transform.rotation = Quaternion.Slerp(palm.transform.rotation, palmQuat, Time.deltaTime * 10f);
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
