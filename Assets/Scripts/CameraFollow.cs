using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;               
    public Vector3 positionOffset = new Vector3(0.7f, 0.8217f, 0f);  // Offset from target
    public Vector3 rotationEuler = new Vector3(30, -90, 0);   // Rotation angle

    public float followSpeed = 5f;         

    void LateUpdate()
    {
        if (target == null) return;

        Vector3 desiredPosition = target.position + positionOffset;
        transform.position = Vector3.Lerp(transform.position, desiredPosition, followSpeed * Time.deltaTime);

        transform.rotation = Quaternion.Euler(rotationEuler);
    }
}
