using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Globalization;

public class UnityReceiver : MonoBehaviour
{
    // Python -> Unity
    public int listenPort = 5015;

    // Unity -> Python (echo + wall state)
    public string echoIp = "127.0.0.1";
    public int echoPort = 5016;

    UdpClient rxClient;
    UdpClient txClient;

    public Transform mcp, pip, dip;
    public float flexFactor = 0f;

    float maxFlex = 1f;
    bool insideWall = false;

    public bool showDebug = true;
    long lastSeq = -1;
    double lastUnityRecvMs = 0;

    void Start()
    {
        rxClient = new UdpClient(new IPEndPoint(IPAddress.Any, listenPort));
        rxClient.Client.Blocking = false;

        txClient = new UdpClient();
        txClient.Connect(echoIp, echoPort);

        Debug.Log("[UnityReceiver] Expecting: seq,send_ms,value");
    }

    void Update()
    {
        while (rxClient != null && rxClient.Available > 0)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = rxClient.Receive(ref anyIP);
                string text = Encoding.UTF8.GetString(data).Trim();

                string[] parts = text.Split(',');
                if (parts.Length == 3 &&
                    long.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out long seq) &&
                    double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double sendMs) &&
                    float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
                {
                    flexFactor = v;
                    double unityRecvMs = Time.realtimeSinceStartupAsDouble * 1000.0;
                    lastSeq = seq;
                    lastUnityRecvMs = unityRecvMs;

                    // IMPORTANT: send back vmax and wall state for Test C
                    // payload: seq,send_ms,unity_recv_ms,vmax,insideWall
                    string echo = string.Format(
                        CultureInfo.InvariantCulture,
                        "{0},{1:F3},{2:F3},{3:F4},{4}",
                        seq, sendMs, unityRecvMs, maxFlex, insideWall ? 1 : 0
                    );
                    byte[] echoBytes = Encoding.UTF8.GetBytes(echo);
                    try { txClient.Send(echoBytes, echoBytes.Length); } catch { }
                }
            }
            catch { }
        }

        // Your wall logic kept
        if (insideWall && flexFactor < 0.1f)
        {
            insideWall = false;
            maxFlex = 1f;
        }

        float clamped = Mathf.Clamp(flexFactor, 0f, maxFlex);
        if (mcp != null) mcp.localRotation = Quaternion.Euler(0, 0, clamped * 120.2f);
        if (pip != null) pip.localRotation = Quaternion.Euler(0, 0, clamped * 117.5f);
        if (dip != null) dip.localRotation = Quaternion.Euler(0, 0, clamped * 76.5f);
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            insideWall = true;
            maxFlex = Mathf.Max(0f, flexFactor - 0.02f);
        }
    }

   // void OnTriggerStay(Collider other)
   // {
   //     if (other.CompareTag("Wall"))
   //     {
   //         if (flexFactor < maxFlex)
   //             maxFlex = Mathf.Max(0f, flexFactor - 0.02f);
   //     }
    //}

void OnTriggerStay(Collider other)
{
    if (other.CompareTag("Wall"))
    {
        insideWall = true;
        if (flexFactor > maxFlex)
            maxFlex = Mathf.Max(0f, flexFactor - 0.02f);
    }
}

    void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            insideWall = false;
            maxFlex = 1f;
        }
    }

    void OnDestroy()
    {
        try { rxClient?.Close(); } catch { }
        try { txClient?.Close(); } catch { }
    }
}