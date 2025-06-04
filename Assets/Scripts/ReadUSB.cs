using System;
using System.IO.Ports;
using UnityEngine;

public class ReadUSB : MonoBehaviour
{
    public string receivedString;
    public float x;
    public float y;
    public float vx;
    public float vy;
    public float button;
    private Vector2 puckPosition;
    private Vector2 puckVelocity;
    private bool buttonPressed;

    const int baudrate = 115200;
    const string PortName = "/dev/tty.usbserial-A10PDHUX";

    SerialPort port = new SerialPort(PortName, baudrate);

    void Start()
    {
        try
        {
            port.Open();
            port.ReadTimeout = 1000;  // Optional: Prevents blocking forever
            //Debug.Log("Serial Open");
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to open serial port: {e.Message}");
        }
    }

    void Update()
    {
        if (port.IsOpen)
        {
            try
            {
                receivedString = port.ReadLine();
                string[] parts = receivedString.Split(',');
                //Debug.Log($"Received line: {receivedString}");
                //Debug.Log($"Parts: {string.Join(", ", parts)}");



                if (parts.Length >= 3)
                {
                    x = float.Parse(parts[0].Trim());
                    y = float.Parse(parts[1].Trim());
                    button = float.Parse(parts[2].Trim());
                    vx = float.Parse(parts[3].Trim());
                    vy = float.Parse(parts[4].Trim());


                    puckPosition = new Vector2(x, y);
                    puckVelocity = new Vector2(vx, vy);
                    buttonPressed = button == 1f;
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Serial read error: {e.Message}");
            }
        }
    }

    public Vector2 GetPuckPosition()
    {
        return puckPosition;
    }

    public Vector2 GetPuckVelocity()
    {
        return puckVelocity;
    }

    public bool GetButtonStatus()
    {
        return buttonPressed;
    }
}
