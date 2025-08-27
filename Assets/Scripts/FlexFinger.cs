using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class FlexFinger : MonoBehaviour
{
    public Transform shoulder;
    public Transform elbow;
    public Transform palm;
    public Transform thumb;
    public Transform index;
    public Transform middle;
    public Transform ring;
    public Transform pinky;

    public Transform obj;
    private Vector3 map;
    private Vector3 mapping;
    public ESP32BleReceiver bleReceiver;
    public Shoulder shoulderReceiver;
    public Elbow elbowReceiver;
    public Glove gloveReceiver;

    private void Start()
    {
        mapping = palm.position;
        map = obj.transform.position;
    }
    void Update()
    {
        handdleQuaternion();
        if(bleReceiver.IsElbowPrinting())
        { Mapping(); }
    }

    private void Mapping()
    {
        Vector3 pos = obj.transform.position;
        pos.x = map.x + (palm.position.x - mapping.x) * 80f;
        pos.y = map.y + (palm.position.y - mapping.y) * 30f;
        obj.transform.position = pos;
    }
    private void handdleQuaternion()
    {
        if (bleReceiver != null)
        {
            string fingerMessage = bleReceiver.GetMessage();
            Quaternion[] palmQuats = bleReceiver.GetPalmQuaternions();
            Quaternion elbowQuat = bleReceiver.GetElbowQuaternion();
            Quaternion shoulderQuat = bleReceiver.GetShoulderQuaternion();

            shoulder.rotation = Quaternion.Slerp(shoulder.rotation, shoulderQuat, Time.deltaTime * 20f);
            elbow.rotation = Quaternion.Slerp(elbow.rotation, elbowQuat, Time.deltaTime * 10f);

            palm.rotation = Quaternion.Slerp(palm.rotation, palmQuats[0], Time.deltaTime * 100f);
            thumb.rotation = Quaternion.Slerp(thumb.rotation, palmQuats[5], Time.deltaTime * 100f);
            index.rotation = Quaternion.Slerp(index.rotation, palmQuats[4], Time.deltaTime * 100f);
            middle.rotation = Quaternion.Slerp(middle.rotation, palmQuats[3], Time.deltaTime * 100f);
            ring.rotation = Quaternion.Slerp(ring.rotation, palmQuats[2], Time.deltaTime * 100f);
            pinky.rotation = Quaternion.Slerp(pinky.rotation, palmQuats[1], Time.deltaTime * 100f);

            //Debug.Log("Palm Quaternion: " + palmQuat);

            if (fingerMessage == null || fingerMessage.Length == 0)
            {
                return;
            }
            else
            {
                //if (fingerMessage.StartsWith("ADC:") && float.TryParse(fingerMessage.Substring(4), out float value))
                //{
                //    index1.transform.localRotation = Quaternion.Euler(0f, 0f, MapSensorToAngle(value));
                //    //Debug.Log("Flex: " + value);
                //}
                //else
                //{
                //    Debug.LogWarning("Invalid ADC format: " + fingerMessage);
                //}
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
