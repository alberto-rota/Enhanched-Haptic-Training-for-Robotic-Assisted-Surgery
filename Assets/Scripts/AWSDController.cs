using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class AWSDController : MonoBehaviour
{
    [Range(0,0.5f)]
    public float scale = 0.001f;
    public Vector3 W;
    public Vector3 A;
    public Vector3 P;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         Rigidbody rb = GetComponent<Rigidbody>();
         if (Input.GetKey(KeyCode.W))
             rb.AddForce(scale*W);
         if (Input.GetKey(KeyCode.S))
             rb.AddForce(scale*W*-1);
         if (Input.GetKey(KeyCode.A))
             rb.AddForce(scale*A);
         if (Input.GetKey(KeyCode.D))
             rb.AddForce(scale*A*-1);
         if (Input.GetKey(KeyCode.P))
             rb.AddForce(scale*P);
         if (Input.GetKey(KeyCode.L))
             rb.AddForce(scale*P*-1);
    }
}
