using System;
using System.Numerics;
using System.Text;
using UnityEngine;
using static BleApi;

public class ESP32Device
{
    public string DeviceName { get; private set; }
    public string DeviceId { get; private set; }
    public bool IsSubscribed { get; private set; }

    private string serviceUuid;
    private string characteristicUuid;

    private bool scanningServices = false;
    private bool scanningCharacteristics = false;

    public Action<float[]> OnQuaternionReceived;
    public Action<int> OnFlexReceived;

    private float[] q = new float[4];
    private string msg = string.Empty;
    public float[] GetQuaternion()
    {
        return q;
    }
    public string GetMessage()
    {
        return msg;
    }

    public ESP32Device(string name, string serviceUuid, string characteristicUuid)
    {
        this.DeviceName = name;
        this.serviceUuid = serviceUuid;
        this.characteristicUuid = characteristicUuid;
    }

    public void CheckDevice(BleApi.DeviceUpdate update)
    {
        if (update.name == DeviceName && DeviceId == null)
        {
            DeviceId = update.id;
            BleApi.ScanServices(DeviceId);
            scanningServices = true;
            Debug.Log($"[{DeviceName}] Found and scanning services.");
        }
    }

    public void UpdateServiceScan()
    {
        if (!scanningServices || string.IsNullOrEmpty(DeviceId))
            return;

        BleApi.Service service = new BleApi.Service();
        while (BleApi.PollService(out service, false) == ScanStatus.AVAILABLE)
        {
            BleApi.ScanCharacteristics(DeviceId, serviceUuid);
            scanningServices = false;
            scanningCharacteristics = true;
            Debug.Log($"[{DeviceName}] Found service, scanning characteristics.");
        }
    }

    public void UpdateCharacteristicScan()
    {
        if (!scanningCharacteristics || string.IsNullOrEmpty(DeviceId))
            return;

        BleApi.Characteristic characteristic;
        while (BleApi.PollCharacteristic(out characteristic, false) == ScanStatus.AVAILABLE)
        {
            BleApi.SubscribeCharacteristic(DeviceId, serviceUuid, characteristicUuid, false);
            scanningCharacteristics = false;
            IsSubscribed = true;
            Debug.Log($"[{DeviceName}] Subscribed to characteristic.");
        }
    }

    public void HandleData(BleApi.BLEData data)
    {
        if (data.deviceId != DeviceId)
            return;

        if (data.size == 16)
        {
            for (int i = 0; i < 4; i++)
                q[i] = BitConverter.ToSingle(data.buf, i * 4);
            OnQuaternionReceived?.Invoke(q);
        }
        else if (data.size > 0 && data.size < 32)
        {
            try
            {
                msg = Encoding.ASCII.GetString(data.buf, 0, data.size);
                if (msg.StartsWith("ADC:") && int.TryParse(msg.Substring(4), out int val))
                {
                    OnFlexReceived?.Invoke(val);
                }
                else
                {
                    Debug.LogWarning($"[{DeviceName}] Unknown text: \"{msg}\"");
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[{DeviceName}] Parse Error: {e.Message}");
            }
        }
        else
        {
            Debug.LogWarning($"[{DeviceName}] Unknown data size: {data.size}");
        }
    }
}
