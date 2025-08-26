using UnityEngine;

public class DragHandler : MonoBehaviour
{
    private bool isDragging = false;
    private ESP32BleReceiver bleReceiver;  // Reference to your BLE data manager
    private bool wasDraggingLastFrame = false;
    private float checkRadius = 0.05f;
    public Rigidbody rb;
    private Vector3 dragOffset = Vector3.zero;

    // Assign which finger/hand transform to follow while dragging
    public Transform handTransformToFollow;

    private void Start()
    {
        bleReceiver = FindObjectOfType<ESP32BleReceiver>();
        rb = GetComponent<Rigidbody>();
        rb.freezeRotation = true;

        if (handTransformToFollow == null)
        {
            Debug.LogWarning("DragHandler: handTransformToFollow not assigned. Assign a Transform to follow during drag.");
        }
    }

    private void OnDestroy()
    {
        if (isDragging && bleReceiver != null)
        {
            // You might implement unregistering logic if needed here
            isDragging = false;
        }
    }

    void Update()
    {
        bool currentlyOverlapping = false;

        Collider[] hits = Physics.OverlapSphere(transform.position, checkRadius);
        foreach (var hit in hits)
        {
            if (hit.CompareTag("Hand"))
            {
                currentlyOverlapping = true;
                break;
            }
        }

        if (currentlyOverlapping && !wasDraggingLastFrame)
        {
            isDragging = true;
            if (handTransformToFollow != null)
                dragOffset = transform.position - handTransformToFollow.position;
            // Optionally notify BLE receiver or other systems that dragging started
        }
        else if (!currentlyOverlapping && wasDraggingLastFrame)
        {
            isDragging = false;
            // Optionally notify BLE receiver or other systems that dragging stopped
        }

        if (isDragging && handTransformToFollow != null)
        {
            Vector3 targetPosition = handTransformToFollow.position + dragOffset;
            rb.useGravity = false;
            rb.MovePosition(targetPosition);
        }
        else
        {
            rb.useGravity = true;
        }

        wasDraggingLastFrame = currentlyOverlapping;
    }
}
