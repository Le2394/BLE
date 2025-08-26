using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using static BleApi;

public class Elbow : MonoBehaviour
{
    private string deviceNameElbow = "DJTME_BLUETOOTH_ELBOW";

    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";

    private string deviceIdElbow = null;
    private bool isScanningDevices = false;

    private bool isScanningServicesElbow = false;

    private bool isScanningCharacteristicsElbow = false;

    private bool subscribedElbow = false;

    private Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();

    private Quaternion elbowQuaternion;

    public Quaternion GetElbowQuaternion()
    {
        return elbowQuaternion;
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

                    if (devName == deviceNameElbow && deviceIdElbow == null)
                    {
                        deviceIdElbow = res.id;
                        BleApi.ScanServices(deviceIdElbow);
                        isScanningServicesElbow = true;
                        Debug.Log("Found Elbow device, scanning services...");
                    }
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }
    }
    void ScanServices()
    {
        BleApi.Service service;

        if (isScanningServicesElbow)
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
    }
    void ScanCharacteristics()
    {
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
    }
    private void PollData()
    {
        if (subscribedElbow)
        {
            BleApi.BLEData data = new BleApi.BLEData();
            while (BleApi.PollData(out data, false))
            {
                if (deviceIdElbow != null && data.deviceId == deviceIdElbow && data.size == 16)
                {
                    float w = BitConverter.ToSingle(data.buf, 0);
                    float x = BitConverter.ToSingle(data.buf, 4);
                    float y = BitConverter.ToSingle(data.buf, 8);
                    float z = BitConverter.ToSingle(data.buf, 12);

                    elbowQuaternion = new Quaternion(x, z, y, -w);
                    Debug.Log($"Elbow Quaternion: {elbowQuaternion}");
                }
            }
        }
    }
    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
