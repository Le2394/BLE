using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using static BleApi;

public class ESP32BleReceiver : MonoBehaviour
{
    //private string deviceNameFlex = "DJTME_BLUETOOTH_FLEX";
    private string deviceNamePalm = "DJTME_BLUETOOTH_PALM";
    private string deviceNameElbow = "DJTME_BLUETOOTH_ELBOW";
    private string deviceNameShoulder = "DJTME_BLUETOOTH_SHOULDER";

    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";


    private string deviceIdFlex = null;
    private string deviceIdPalm = null;
    private string deviceIdElbow = null;
    private string deviceIdShoulder = null;     

    private bool isScanningDevices = false;

    private bool isScanningServicesFlex = false;
    private bool isScanningServicesPalm = false;
    private bool isScanningServicesElbow = false;
    private bool isScanningServicesShoulder = false;

    private bool isScanningCharacteristicsFlex = false;
    private bool isScanningCharacteristicsPalm = false;
    private bool isScanningCharacteristicsElbow = false;
    private bool isScanningCharacteristicsShoulder = false;

    private bool subscribedFlex = false;
    private bool subscribedPalm = false;
    private bool subscribedElbow = false;
    private bool subscribedShoulder = false;

    private bool palmIsPrinting = false;
    private bool elbowIsPrinting = false;
    private bool shoulderIsPrinting = false;

    private Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();

    private string msg;
    private Quaternion[] palmQuaternions = new Quaternion[6];
    private Quaternion elbowQuaternion;
    private Quaternion shoulderQuaternion;

    private int adcValue = 0;
    public string GetMessage() => msg;
    public bool IsPalmPrinting() => palmIsPrinting;
    public bool IsElbowPrinting() => elbowIsPrinting;
    public void SetPalmQuaternions(Quaternion[] quats)
    {
        for (int i = 0; i < 6; i++)
        {
            palmQuaternions[i] = quats[i];
        }
    }

    public Quaternion[] GetPalmQuaternions()
    {
        return palmQuaternions;
    }

    public Quaternion GetElbowQuaternion()
    {
        return elbowQuaternion;
    }

    public Quaternion GetShoulderQuaternion()
    {
        return shoulderQuaternion;
    }

    void Start()
    {
        BleApi.StartDeviceScan();
        Debug.Log("Started scanning...");
        isScanningDevices = true;
    }
    void Update()
    {
        ScanDevices();
        ScanServices();
        ScanCharacteristics();
        PollData();
    }
    public void RestartScene()
    {
        BleApi.Quit();
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
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

                    if (devName == deviceNamePalm && deviceIdPalm == null)
                    {
                        deviceIdPalm = res.id;
                        BleApi.ScanServices(deviceIdPalm);
                        isScanningServicesPalm = true;
                        Debug.Log("Found Palm device, scanning services...");
                    }
                    if (devName == deviceNameElbow && deviceIdElbow == null)
                    {
                        deviceIdElbow = res.id;
                        BleApi.ScanServices(deviceIdElbow);
                        isScanningServicesElbow = true;
                        Debug.Log("Found Elbow device, scanning services...");
                    }
                    if (devName == deviceNameShoulder && deviceIdShoulder == null)
                    {
                        deviceIdShoulder = res.id;
                        BleApi.ScanServices(deviceIdShoulder);
                        isScanningServicesShoulder = true;
                        Debug.Log("Found Shoulder device, scanning services...");
                    }
                    //if (devName == deviceNameFlex && deviceIdFlex == null)
                    //{
                    //    deviceIdFlex = res.id;
                    //    BleApi.ScanServices(deviceIdFlex);
                    //    isScanningServicesFlex = true;
                    //    Debug.Log("Found Flex device, scanning services...");
                    //}

                    //if ((deviceIdFlex != null || deviceIdS3 != null) || deviceIdElbow != null)
                    //{
                    //    BleApi.StopDeviceScan();
                    //    isScanningDevices = false;
                    //}
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }
    }
    void ScanServices()
    {
        BleApi.Service service;

        if (isScanningServicesPalm)
        {
            while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
            {
                BleApi.ScanCharacteristics(deviceIdPalm, service.uuid);
                isScanningCharacteristicsPalm = true;
                isScanningServicesPalm = false;
                Debug.Log($"Palm Found service UUID: {service.uuid}");
                break;
            }
        }
        else if (isScanningServicesElbow)
        {
            while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
            {
                BleApi.ScanCharacteristics(deviceIdElbow, service.uuid);
                isScanningCharacteristicsElbow = true;
                isScanningServicesElbow = false;
                Debug.Log($"Elbow Found service UUID: {service.uuid}");
                break;
            }
        }
        else if (isScanningServicesShoulder)
        {
            while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
            {
                BleApi.ScanCharacteristics(deviceIdShoulder, service.uuid);
                isScanningCharacteristicsShoulder = true;
                isScanningServicesShoulder = false;
                Debug.Log($"Shoulder Found service UUID: {service.uuid}");
                break;
            }
        }
        //else if (isScanningServicesFlex)
        //{
        //    while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
        //    {
        //        BleApi.ScanCharacteristics(deviceIdFlex, service.uuid);
        //        isScanningCharacteristicsFlex = true;
        //        isScanningServicesFlex = false;
        //        Debug.Log($"Flex Found service UUID: {service.uuid}");
        //        break;
        //    }
        //}
    }
    void ScanCharacteristics()
    {
        if (isScanningCharacteristicsShoulder)
        {
            BleApi.Characteristic characteristic;
            while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
            {
                BleApi.SubscribeCharacteristic(deviceIdShoulder, serviceUuid, characteristicUuid, false);
                subscribedShoulder = true;
                isScanningCharacteristicsShoulder = false;
                Debug.Log("Subscribed to Shoulder device");
            }
        }
        if (isScanningCharacteristicsElbow)
        {
            BleApi.Characteristic characteristic;
            while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
            {
                BleApi.SubscribeCharacteristic(deviceIdElbow, serviceUuid, characteristicUuid, false);
                subscribedElbow = true;
                isScanningCharacteristicsElbow = false;
                Debug.Log("Subscribed to Elbow device");
            }
        }        
        if (isScanningCharacteristicsPalm)
        {
            BleApi.Characteristic characteristic;
            while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
            {
                BleApi.SubscribeCharacteristic(deviceIdPalm, serviceUuid, characteristicUuid, false);
                subscribedPalm = true;
                isScanningCharacteristicsPalm = false;
                Debug.Log("Subscribed to Palm device");
            }
        }
        //if (isScanningCharacteristicsFlex)
        //{
        //    BleApi.Characteristic characteristic;
        //    while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
        //    {
        //        BleApi.SubscribeCharacteristic(deviceIdFlex, serviceUuid, characteristicUuid, false);
        //        subscribedFlex = true;
        //        isScanningCharacteristicsFlex = false;
        //        Debug.Log("Subscribed to Flex device");
        //    }
        //}
    }
    private void PollData()
    {
        if (subscribedFlex || subscribedPalm || subscribedElbow || subscribedShoulder)
        {
            BleApi.BLEData data = new BleApi.BLEData();
            while (BleApi.PollData(out data, false))
            {
                if (deviceIdPalm != null && data.deviceId == deviceIdPalm && data.size == 96)
                {
                    Quaternion[] quats = new Quaternion[6];
                    for (int i = 0; i < 6; i++)
                    {
                        int offset = i * 16;
                        float w = BitConverter.ToSingle(data.buf, offset + 0);
                        float x = BitConverter.ToSingle(data.buf, offset + 4);
                        float y = BitConverter.ToSingle(data.buf, offset + 8);
                        float z = BitConverter.ToSingle(data.buf, offset + 12);
                        if (i == 0)
                        {
                            quats[i] = new Quaternion(x, z, y, w);
                        }
                        if (i == 3)
                        {
                            quats[i] = quats[i-1];
                        }
                        else
                        {
                            quats[i] = new Quaternion(x, 0, 0, w);
                        }
                        //Debug.Log($"Palm IMU {i} Quaternion: {quats[i]}");
                    }
                    palmIsPrinting = true;
                    SetPalmQuaternions(quats);
                }
                else if (deviceIdElbow != null && data.deviceId == deviceIdElbow && data.size == 16)
                {
                    float w = BitConverter.ToSingle(data.buf, 0);
                    float x = BitConverter.ToSingle(data.buf, 4);
                    float y = BitConverter.ToSingle(data.buf, 8);
                    float z = BitConverter.ToSingle(data.buf, 12);

                    elbowQuaternion = new Quaternion(x, z, y, w);
                    Debug.Log($"Elbow Quaternion: {elbowQuaternion}");
                    elbowIsPrinting = true;
                }
                else if (deviceIdShoulder != null && data.deviceId == deviceIdShoulder && data.size == 16)
                {
                    float w = BitConverter.ToSingle(data.buf, 0);
                    float x = BitConverter.ToSingle(data.buf, 4);
                    float y = BitConverter.ToSingle(data.buf, 8);
                    float z = BitConverter.ToSingle(data.buf, 12);

                    shoulderQuaternion = new Quaternion(x, z, y, -w); // Axis adjustment
                }
                //if (deviceIdFlex != null && data.deviceId == deviceIdFlex)
                //{
                //    if (data.size == 16)
                //    {
                //        float fq0 = BitConverter.ToSingle(data.buf, 0);
                //        float fq1 = BitConverter.ToSingle(data.buf, 4);
                //        float fq2 = BitConverter.ToSingle(data.buf, 8);
                //        float fq3 = BitConverter.ToSingle(data.buf, 12);
                //        Debug.Log($"Quaternion: {fq0:F2}, {fq1:F2}, {fq2:F2}, {fq3:F2}");
                //    }
                //    else if (data.size > 0 && data.size < 32)
                //    {
                //        try
                //        {
                //            msg = Encoding.ASCII.GetString(data.buf, 0, data.size);
                //            if (msg.StartsWith("ADC:") && int.TryParse(msg.Substring(4), out int val))
                //            {
                //                adcValue = val;
                //                Debug.Log($"Flex: {adcValue}");
                //            }
                //            else
                //            {
                //                Debug.LogWarning($"Unknown Text: \"{msg}\" ({data.size} bytes)");
                //            }
                //        }
                //        catch (Exception e)
                //        {
                //            Debug.LogWarning($"Flex Text Parse Error Size: {data.size} | {e.Message}");
                //        }
                //    }
                //    else
                //    {
                //        Debug.LogWarning($"Unhandled data size: {data.size}");
                //    }
                //}
            }
        }
    }
    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
