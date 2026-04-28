using UnityEngine;
using UnityEngine.InputSystem;

public class Moveable : MonoBehaviour
{
    private float zDepth;
    private Camera mainCamera;
    private Transform tr;
    private Mouse mouse;

    private void Awake()
    {
        tr = transform;
        mainCamera = Camera.main;
        mouse = Mouse.current;
    }

    private void OnMouseDown()
    {
        if (mainCamera == null || mouse == null) return;

        // keep z-depth constant
        zDepth = mainCamera.WorldToScreenPoint(tr.position).z;
    }

    private void OnMouseDrag()
    {
        if (mainCamera == null || mouse == null) return;

        var pos = mouse.position.ReadValue();
        tr.position = mainCamera.ScreenToWorldPoint(new Vector3(pos.x, pos.y, zDepth));
    }
}
