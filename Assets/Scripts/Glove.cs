using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using static BleApi;

public class Glove : MonoBehaviour
{
    private string deviceNamePalm = "DJTME_BLUETOOTH_PALM";

    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";

    private string deviceIdPalm = null;

    private bool isScanningDevices = false;

    private bool isScanningServicesPalm = false;

    private bool isScanningCharacteristicsPalm = false;

    private bool subscribedPalm = false;

    private Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();

    private Quaternion[] palmQuaternions = new Quaternion[6];

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
    }
    void ScanCharacteristics()
    {
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

    }
    private void PollData()
    {
        if (subscribedPalm)
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
                            quats[i] = new Quaternion(x, -z, y, w);
                        }
                        else
                        {
                            quats[i] = new Quaternion(x, 0, 0, w);
                        }
                        Debug.Log($"Palm IMU {i} Quaternion: {quats[i]}");
                    }
                    SetPalmQuaternions(quats);
                }
            }
        }
    }
    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
