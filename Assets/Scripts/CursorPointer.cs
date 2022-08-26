using UnityEngine;
using System.Collections;

public class CursorPointer : MonoBehaviour
{
    public float offset_3d = 0;
    void Update() {
        gameObject.transform.position = Input.mousePosition + new Vector3(offset_3d,0,0);
    }

}