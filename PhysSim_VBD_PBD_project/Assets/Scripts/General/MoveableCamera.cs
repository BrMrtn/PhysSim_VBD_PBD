using UnityEngine;
using UnityEngine.InputSystem;

public class MoveableCamera : MonoBehaviour
{
    public float moveSpeed = 10f;
    public float lookSpeed = 2f;

    float yaw;
    float pitch;

    void Start()
    {
        Vector3 euler = transform.eulerAngles;
        yaw = euler.y;
        pitch = euler.x;
    }

    void Update()
    {
        var mouse = Mouse.current;
        var keyboard = Keyboard.current;

        if (mouse == null || keyboard == null) return;

        // Look (Right Mouse Button)
        if (mouse.rightButton.isPressed)
        {
            Vector2 delta = mouse.delta.ReadValue();

            yaw += delta.x * lookSpeed * 0.1f;
            pitch -= delta.y * lookSpeed * 0.1f;
            pitch = Mathf.Clamp(pitch, -89f, 89f);

            transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
        }

        // Movement
        Vector3 move = Vector3.zero;

        if (keyboard.wKey.isPressed) move += transform.forward;
        if (keyboard.sKey.isPressed) move -= transform.forward;
        if (keyboard.dKey.isPressed) move += transform.right;
        if (keyboard.aKey.isPressed) move -= transform.right;
        if (keyboard.eKey.isPressed) move += transform.up;
        if (keyboard.qKey.isPressed) move -= transform.up;

        transform.position += move.normalized * moveSpeed * Time.deltaTime;
    }
}
