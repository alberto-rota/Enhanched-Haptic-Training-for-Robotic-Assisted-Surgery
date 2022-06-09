using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class AWSDController : MonoBehaviour
{
    [Range(0,0.5f)]
    public float scale = 0.001f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         Rigidbody rb = GetComponent<Rigidbody>();
         if (Input.GetKey(KeyCode.W))
             rb.AddForce(scale*Vector3.left);
         if (Input.GetKey(KeyCode.S))
             rb.AddForce(scale*Vector3.right);
         if (Input.GetKey(KeyCode.A))
             rb.AddForce(scale*Vector3.back);
         if (Input.GetKey(KeyCode.D))
             rb.AddForce(scale*Vector3.forward);
         if (Input.GetKey(KeyCode.P))
             rb.AddForce(scale*Vector3.up);
         if (Input.GetKey(KeyCode.L))
             rb.AddForce(scale*Vector3.down);
    }
}
