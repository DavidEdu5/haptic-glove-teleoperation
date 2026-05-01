using UnityEngine;

public class WallDetector : MonoBehaviour
{
    void OnTriggerEnter(Collider other)
    {
        Debug.Log("HIT BY: " + other.name);
    }

    void OnTriggerExit(Collider other)
    {
        Debug.Log("LEFT WALL: " + other.name);
    }
}