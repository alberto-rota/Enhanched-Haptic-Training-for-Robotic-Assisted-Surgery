using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AWSDController : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         Rigidbody rb = GetComponent<Rigidbody>();
         if (Input.GetKey(KeyCode.W))
             rb.AddForce(Vector3.left);
         if (Input.GetKey(KeyCode.S))
             rb.AddForce(Vector3.right);
         if (Input.GetKey(KeyCode.A))
             rb.AddForce(Vector3.back);
         if (Input.GetKey(KeyCode.D))
             rb.AddForce(Vector3.forward);
         if (Input.GetKey(KeyCode.P))
             rb.AddForce(Vector3.up);
         if (Input.GetKey(KeyCode.L))
             rb.AddForce(Vector3.down);
    }
}
