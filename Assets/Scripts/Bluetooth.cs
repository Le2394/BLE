using System;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using static BleApi;

public class ESP32BleReceiver : MonoBehaviour
{
    private string deviceNameFlex = "DJTME_BLUETOOTH_FLEX";
    private string deviceNameS3 = "DJTME_BLUETOOTH_S3";
    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";


    private string deviceIdFlex = null;
    private string deviceIdS3 = null;

    private bool isScanningDevices = false;
    private bool isScanningServicesFlex = false;
    private bool isScanningServicesS3 = false;
    private bool isScanningCharacteristicsFlex = false;
    private bool isScanningCharacteristicsS3 = false;

    private bool subscribedFlex = false;
    private bool subscribedS3 = false;

    private BLEData data = new BLEData();
    private Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();

    private string msg;
    private float q0, q1, q2, q3;
    private int adcValue = 0;

    void Start()
    {
        BleApi.StartDeviceScan();
        Debug.Log("Started scanning...");
        isScanningDevices = true;
    }
    public string GetMessage()
    {
        return msg;
    }

    public float[] GetQuaternion()
    {
        return new float[] { q0, q1, q2, q3 };
    }

    void Update()
    {
        ScanDevices();
        ScanServices();
        ScanCharacteristics();
        if (subscribedFlex || subscribedS3)
        {
            BleApi.BLEData data = new BleApi.BLEData();

            while (BleApi.PollData(out data, false))
            {
                if (deviceIdS3 != null && data.deviceId == deviceIdS3 && data.size == 16)
                {
                    q0 = BitConverter.ToSingle(data.buf, 0);
                    q1 = BitConverter.ToSingle(data.buf, 4);
                    q2 = BitConverter.ToSingle(data.buf, 8);
                    q3 = BitConverter.ToSingle(data.buf, 12);
                    Debug.Log($"S3 Quaternion: {q0:F2}, {q1:F2}, {q2:F2}, {q3:F2}");
                }
                if (deviceIdFlex != null && data.deviceId == deviceIdFlex)
                {
                    if (data.size == 16)
                    {
                        float fq0 = BitConverter.ToSingle(data.buf, 0);
                        float fq1 = BitConverter.ToSingle(data.buf, 4);
                        float fq2 = BitConverter.ToSingle(data.buf, 8);
                        float fq3 = BitConverter.ToSingle(data.buf, 12);
                        //Debug.Log($"Quaternion: {fq0:F2}, {fq1:F2}, {fq2:F2}, {fq3:F2}");
                    }
                    else if (data.size > 0 && data.size < 32)
                    {
                        try
                        {
                            msg = Encoding.ASCII.GetString(data.buf, 0, data.size);
                            if (msg.StartsWith("ADC:") && int.TryParse(msg.Substring(4), out int val))
                            {
                                adcValue = val;
                                Debug.Log($"Flex: {adcValue}");
                            }
                            else
                            {
                                Debug.LogWarning($"Unknown Text: \"{msg}\" ({data.size} bytes)");
                            }
                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning($"Flex Text Parse Error Size: {data.size} | {e.Message}");
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"Unhandled data size: {data.size}");
                    }
                }
            }
        }
    }

    void ScanDevices()
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

                    string devName = devices[res.id]["name"];
                    if (devName == deviceNameFlex && deviceIdFlex == null)
                    {
                        deviceIdFlex = res.id;
                        BleApi.ScanServices(deviceIdFlex);
                        isScanningServicesFlex = true;
                        Debug.Log("Found Flex device, scanning services...");
                    }
                    if (devName == deviceNameS3 && deviceIdS3 == null)
                    {
                        deviceIdS3 = res.id;
                        BleApi.ScanServices(deviceIdS3);
                        isScanningServicesS3 = true;
                        Debug.Log("Found S3 device, scanning services...");
                    }

                    if ((deviceIdFlex != null || deviceIdS3 != null))
                    {
                        BleApi.StopDeviceScan();
                        isScanningDevices = false;
                    }
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }
    }

    void ScanServices()
    {
        if (isScanningServicesFlex)
        {
            BleApi.Service service = new BleApi.Service();
            while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
            {
                BleApi.ScanCharacteristics(deviceIdFlex, serviceUuid);
                isScanningCharacteristicsFlex = true;
                isScanningServicesFlex = false;
                Debug.Log("Found service for Flex device");
            }
        }

        if (isScanningServicesS3)
        {
            BleApi.Service service = new BleApi.Service();
            while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
            {
                BleApi.ScanCharacteristics(deviceIdS3, serviceUuid);
                isScanningCharacteristicsS3 = true;
                isScanningServicesS3 = false;
                Debug.Log("Found service for S3 device");
            }
        }
    }

    void ScanCharacteristics()
    {
        if (isScanningCharacteristicsFlex)
        {
            BleApi.Characteristic characteristic;
            while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
            {
                BleApi.SubscribeCharacteristic(deviceIdFlex, serviceUuid, characteristicUuid, false);
                subscribedFlex = true;
                isScanningCharacteristicsFlex = false;
                Debug.Log("Subscribed to Flex device");
            }
        }

        if (isScanningCharacteristicsS3)
        {
            Debug.Log("Subscribed to S3 device");
            BleApi.Characteristic characteristic;
            while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
            {
                BleApi.SubscribeCharacteristic(deviceIdS3, serviceUuid, characteristicUuid, false);
                subscribedS3 = true;
                isScanningCharacteristicsS3 = false;
                Debug.Log("Subscribed to S3 device");
            }
        }
    }
    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
