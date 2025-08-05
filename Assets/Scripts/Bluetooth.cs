using System;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Text;
using UnityEngine;
using static BleApi;

public class ESP32BleReceiver : MonoBehaviour
{
    private string targetDeviceName = "DJTME_BLUETOOTH";
    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";

    private string selectedDeviceId;
    [SerializeField]
    public bool isScanningDevices = false;
    public bool isScanningServices = false;
    public bool isScanningCharacteristics = false;
    public bool isSubscribed = false;
    private float connectTimer = 0f;
    private float connectDelay = 2f;
    Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();
    private string msg;
    private BleApi.BLEData data = new BleApi.BLEData();

    private float q0, q1, q2, q3;
    public string GetMessage()
    {
        return msg;
    }

    public float[] GetQuaternion()
    {
        return new float[] { q0, q1, q2, q3 };
    }

    void Start()
    {
        BleApi.StartDeviceScan();
        Debug.Log("Started scanning...");
        isScanningDevices = true;
    }

   void Update()
    {
        BleApi.ScanStatus status;
        if (isScanningDevices)
        {
            BleApi.DeviceUpdate res = new BleApi.DeviceUpdate();
            do
            {
                status = BleApi.PollDevice(ref res, false);
                if (status == BleApi.ScanStatus.AVAILABLE)
                {
                    if (!devices.ContainsKey(res.id))
                        devices[res.id] = new Dictionary<string, string>() {
                            { "name", "" },
                            { "isConnectable", "False" }
                        };
                    if (res.nameUpdated)
                        devices[res.id]["name"] = res.name;
                    if (res.isConnectableUpdated)
                        devices[res.id]["isConnectable"] = res.isConnectable.ToString();
                    // consider only devices which have a name and which are connectable
                    if (devices[res.id]["name"] == targetDeviceName)
                    {
                        Debug.Log($"Scanning Device: {res.name} ({res.id}) - Connectable: {res.isConnectable}");
                        selectedDeviceId = res.id;
                        BleApi.StopDeviceScan();
                        isScanningDevices = false;

                        BleApi.ScanServices(selectedDeviceId);
                        isScanningServices = true;
                    }
                }
                else if (status == BleApi.ScanStatus.FINISHED)
                {
                    isScanningDevices = false;
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }

        if (isScanningServices)
        {
            BleApi.Service res = new BleApi.Service();
            do
            {
                status = BleApi.PollService(out res, false);
                if (status == BleApi.ScanStatus.AVAILABLE)
                {
                    Debug.Log($"Service: {res.uuid}");
                    Debug.Log(res.uuid.ToLower() + " == " + serviceUuid.ToLower());
                    Debug.Log("Service matched. Scanning characteristics...");
                    BleApi.ScanCharacteristics(selectedDeviceId, serviceUuid);
                    isScanningServices = false;
                    isScanningCharacteristics = true;
                }
                else if (status == BleApi.ScanStatus.FINISHED)
                {
                    isScanningServices = false;
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);

        }
        if (isScanningCharacteristics)
        {
            BleApi.Characteristic res = new BleApi.Characteristic();
            do
            {
                status = BleApi.PollCharacteristic(out res, false);
                if (status == BleApi.ScanStatus.AVAILABLE)
                {
                    Debug.Log($"Characteristic: {res.uuid}");
                    Debug.Log("Characteristic matched. Subscribing...");
                    BleApi.SubscribeCharacteristic(selectedDeviceId, serviceUuid, characteristicUuid, false);
                    isScanningCharacteristics = false;
                    isSubscribed = true;

                }
                else if (status == BleApi.ScanStatus.FINISHED)
                {
                    isScanningCharacteristics = false;
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }
        
        if (isSubscribed)
        {
            while (BleApi.PollData(out data, false))
            {
                if (data.size == 16)
                {
                    q0 = BitConverter.ToSingle(data.buf, 0);
                    q1 = BitConverter.ToSingle(data.buf, 4);
                    q2 = BitConverter.ToSingle(data.buf, 8);
                    q3 = BitConverter.ToSingle(data.buf, 12);

                    //Debug.Log($"{q0:F3}, {q1:F3}, {q2:F3}, {q3:F3}");
                }
                else
                {
                    string msg = Encoding.ASCII.GetString(data.buf, 0, data.size);
                    if (msg.StartsWith("ADC:"))
                    {
                        if (int.TryParse(msg.Substring(4), out int adcValue))
                        {
                            Debug.Log($"[ADC] Value: {adcValue}");
                        }
                        else
                        {
                            Debug.LogWarning("Failed to parse ADC value: " + msg);
                        }
                    }
                    else
                    {
                        Debug.LogWarning("Received unknown format or unexpected byte size: " + data.size);
                    }
                }
            }
        }

        //BleApi.ErrorMessage err;
        //BleApi.GetError(out err);
        //if (!string.IsNullOrEmpty(err.msg))
        //    Debug.LogError("BLE Error: " + err.msg);
    }

    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
