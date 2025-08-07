using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Balloon : MonoBehaviour
{
    public ESP32BleReceiver bleReceiver;
    Vector3 origScale;
    public Vector3 scaler = new Vector3(1, 1, 1);
    public float speed = 1f;

    void Start()
    {
        origScale = transform.localScale;
    }

    void Update()
    {
        if (bleReceiver != null)
        {
            string fingerMessage = bleReceiver.GetMessage();
            if (fingerMessage == null || fingerMessage.Length == 0)
            {
                return;
            }
            else
            {
                if (fingerMessage.StartsWith("ADC:") && float.TryParse(fingerMessage.Substring(4), out float value))
                {
                    float normalized = Mathf.InverseLerp(300f, 1100f, value);
                    transform.localScale += scaler * normalized * speed * Time.deltaTime;
                    Debug.Log(value + " " + normalized);
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
