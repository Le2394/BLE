using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.ConstrainedExecution;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using static BleApi;

public class Shoulder : MonoBehaviour
{
    private string deviceNameShoulder = "DJTME_BLUETOOTH_SHOULDER";

    private string serviceUuid = "56781278-1234-5634-3412-34129abcdef0";
    private string characteristicUuid = "78563412-0000-0000-beba-fecaefbeadde";

    private string deviceIdShoulder = null;
    private bool isScanningDevices = false;

    private bool isScanningServicesShoulder = false;

    private bool isScanningCharacteristicsShoulder = false;

    private bool subscribedShoulder = false;

    private Dictionary<string, Dictionary<string, string>> devices = new Dictionary<string, Dictionary<string, string>>();

    private Quaternion shoulderQuaternion;

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

                    if (devName == deviceNameShoulder && deviceIdShoulder == null)
                    {
                        deviceIdShoulder = res.id;
                        BleApi.ScanServices(deviceIdShoulder);
                        isScanningServicesShoulder = true;
                        Debug.Log("Found Shoulder device, scanning services...");
                    }
                }
            } while (status == BleApi.ScanStatus.AVAILABLE);
        }
    }
    void ScanServices()
    {
        BleApi.Service service;

        if (isScanningServicesShoulder)
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
    }
    private void PollData()
    {
        if (subscribedShoulder)
        {
            BleApi.BLEData data = new BleApi.BLEData();
            while (BleApi.PollData(out data, false))
            {
                if (deviceIdShoulder != null && data.deviceId == deviceIdShoulder && data.size == 16)
                {
                    float w = BitConverter.ToSingle(data.buf, 0);
                    float x = BitConverter.ToSingle(data.buf, 4);
                    float y = BitConverter.ToSingle(data.buf, 8);
                    float z = BitConverter.ToSingle(data.buf, 12);

                    shoulderQuaternion = new Quaternion(x, z, y, -w);
                }
            }
        }
    }
    void OnApplicationQuit()
    {
        BleApi.Quit();
    }
}
